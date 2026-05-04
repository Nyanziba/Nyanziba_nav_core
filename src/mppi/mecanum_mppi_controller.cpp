// MecanumMppiController: classic Sample-Based MPPI on a mecanum
// kinematics model. Eigen-only; CasADi-backed dynamics evaluation is a
// future drop-in (will live behind TEXNITIS_NAV_CORE_WITH_CASADI).

#include "texnitis_nav_core/mppi/mecanum_mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace texnitis::nav_core::mppi {

namespace {

/// Pose2D を MPPI の State に詰める。残りの速度成分は呼び出し側が埋める。
State poseToState (const Pose2D &pose, const Eigen::Vector3d &body_vel) noexcept {
    State s = State::Zero ();
    s (0) = pose.x;
    s (1) = pose.y;
    s (2) = pose.yaw;
    s (3) = body_vel (0);
    s (4) = body_vel (1);
    s (5) = body_vel (2);
    return s;
}

}  // namespace

MecanumMppiController::MecanumMppiController (const MppiParams &params) {
    setParams (params);
}

void MecanumMppiController::setParams (const MppiParams &params) {
    params_     = params;
    kinematics_ = MecanumKinematics (params.v_max, params.omega_max, params.dt, params.wheel_base);
    goal_checker_.setParams (params.goal_checker);
    rng_.seed (params.seed != 0 ? params.seed : 42u);
    u_nominal_.assign (static_cast<size_t> (std::max (1, params.horizon)), Control::Zero ());
    body_velocity_.setZero ();
}

void MecanumMppiController::setPlan (const Path2D &path) {
    path_ = path;
    goal_checker_.reset ();
    rng_.seed (params_.seed != 0 ? params_.seed : 42u);
    std::fill (u_nominal_.begin (), u_nominal_.end (), Control::Zero ());
    body_velocity_.setZero ();
    cancel_flag_.store (false, std::memory_order_relaxed);
    last_goal_reached_ = false;
}

void MecanumMppiController::reset () {
    cancel_flag_.store (false, std::memory_order_relaxed);
    goal_checker_.reset ();
    path_.poses.clear ();
    rng_.seed (params_.seed != 0 ? params_.seed : 42u);
    std::fill (u_nominal_.begin (), u_nominal_.end (), Control::Zero ());
    body_velocity_.setZero ();
    last_goal_reached_ = false;
}

size_t MecanumMppiController::nearestIndex (double x, double y) const noexcept {
    size_t best_idx  = 0;
    double best_dist = std::numeric_limits<double>::infinity ();
    for (size_t i = 0; i < path_.poses.size (); ++i) {
        const double dx = path_.poses[i].x - x;
        const double dy = path_.poses[i].y - y;
        const double d  = dx * dx + dy * dy;
        if (d < best_dist) {
            best_dist = d;
            best_idx  = i;
        }
    }
    return best_idx;
}

double MecanumMppiController::rolloutCost (const std::vector<State> &trajectory,
                                            const std::vector<Control> &controls,
                                            size_t                       nearest_path_idx) const noexcept {
    if (path_.poses.empty () || trajectory.empty ()) {
        return std::numeric_limits<double>::infinity ();
    }
    const Pose2D &goal = path_.poses.back ();
    double cost = 0.0;
    size_t path_idx = nearest_path_idx;
    for (size_t step = 0; step < trajectory.size (); ++step) {
        const State &s = trajectory[step];
        // Advance the path index in lock-step with the rollout: pick
        // whichever pose ahead of us is closest.
        double best_d = (path_.poses[path_idx].x - s (0)) * (path_.poses[path_idx].x - s (0)) +
                        (path_.poses[path_idx].y - s (1)) * (path_.poses[path_idx].y - s (1));
        for (size_t j = path_idx + 1; j < path_.poses.size () && j < path_idx + 5; ++j) {
            const double dx = path_.poses[j].x - s (0);
            const double dy = path_.poses[j].y - s (1);
            const double d  = dx * dx + dy * dy;
            if (d < best_d) {
                best_d   = d;
                path_idx = j;
            }
        }
        cost += params_.w_track * best_d;

        const double yaw_err = shortestAngularDiff (s (2), goal.yaw);
        cost += params_.w_yaw * yaw_err * yaw_err;

        if (step < controls.size ()) {
            const Control &u = controls[step];
            cost += params_.w_control * (u (0) * u (0) + u (1) * u (1) + u (2) * u (2));
        }
    }
    // Terminal cost: distance to the goal at the end of the rollout.
    const Pose2D end_pose{trajectory.back () (0), trajectory.back () (1), trajectory.back () (2)};
    cost += params_.w_track * 5.0 * distanceXY (end_pose, goal) * distanceXY (end_pose, goal);
    return cost;
}

ControllerResult MecanumMppiController::computeCommand (const Pose2D &current,
                                                         Twist2D      &cmd_out) {
    cmd_out = Twist2D{};
    if (cancel_flag_.load (std::memory_order_relaxed)) {
        return ControllerResult::Cancelled;
    }
    if (path_.poses.empty ()) {
        return ControllerResult::EmptyPath;
    }
    if (params_.horizon <= 0 || params_.num_samples <= 0) {
        return ControllerResult::NotInitialized;
    }

    const Pose2D goal = path_.poses.back ();
    if (goal_checker_.isReached (current, goal)) {
        last_goal_reached_ = true;
        body_velocity_.setZero ();
        return ControllerResult::GoalReached;
    }

    const State initial_state   = poseToState (current, body_velocity_);
    const size_t nearest_idx    = nearestIndex (current.x, current.y);
    const int    horizon        = params_.horizon;
    const int    num_samples    = params_.num_samples;

    std::vector<std::vector<Control>> all_controls (num_samples,
                                                    std::vector<Control> (horizon));
    std::vector<double> costs (num_samples);

    std::normal_distribution<double> noise_x (0.0, params_.sigma (0));
    std::normal_distribution<double> noise_y (0.0, params_.sigma (1));
    std::normal_distribution<double> noise_w (0.0, params_.sigma (2));

    for (int n = 0; n < num_samples; ++n) {
        std::vector<State> traj;
        traj.reserve (horizon + 1);
        traj.push_back (initial_state);
        for (int t = 0; t < horizon; ++t) {
            Control eps;
            eps (0) = noise_x (rng_);
            eps (1) = noise_y (rng_);
            eps (2) = noise_w (rng_);
            Control u = u_nominal_[t] + eps;
            for (int k = 0; k < 3; ++k) {
                u (k) = std::min (std::max (u (k), -params_.u_max (k)), params_.u_max (k));
            }
            all_controls[n][t] = u;
            traj.push_back (kinematics_.updateState (traj.back (), u));
        }
        costs[n] = rolloutCost (traj, all_controls[n], nearest_idx);
    }

    // softmax weights.
    const double min_cost = *std::min_element (costs.begin (), costs.end ());
    if (!std::isfinite (min_cost)) {
        return ControllerResult::ComputationFailed;
    }
    double weight_sum = 0.0;
    std::vector<double> weights (num_samples);
    for (int n = 0; n < num_samples; ++n) {
        weights[n] = std::exp (-(costs[n] - min_cost) / std::max (1e-9, params_.lambda));
        weight_sum += weights[n];
    }
    if (weight_sum <= 0.0 || !std::isfinite (weight_sum)) {
        return ControllerResult::ComputationFailed;
    }

    // Update nominal control sequence by weighted average.
    for (int t = 0; t < horizon; ++t) {
        Control accum = Control::Zero ();
        for (int n = 0; n < num_samples; ++n) {
            accum += weights[n] * all_controls[n][t];
        }
        u_nominal_[t] = accum / weight_sum;
    }

    // Take the first action and integrate body_velocity_ for next call.
    const Control first_u = u_nominal_[0];
    body_velocity_ += first_u * params_.dt;
    const double speed = std::hypot (body_velocity_ (0), body_velocity_ (1));
    if (speed > params_.v_max) {
        body_velocity_ (0) *= params_.v_max / speed;
        body_velocity_ (1) *= params_.v_max / speed;
    }
    if (std::abs (body_velocity_ (2)) > params_.omega_max) {
        body_velocity_ (2) = std::copysign (params_.omega_max, body_velocity_ (2));
    }

    // Shift nominal sequence one step (warm-start).
    for (int t = 0; t + 1 < horizon; ++t) {
        u_nominal_[t] = u_nominal_[t + 1];
    }
    u_nominal_[horizon - 1] = Control::Zero ();

    cmd_out.vx = body_velocity_ (0);
    cmd_out.vy = body_velocity_ (1);
    cmd_out.wz = body_velocity_ (2);
    last_goal_reached_ = false;
    return ControllerResult::Success;
}

bool MecanumMppiController::isGoalReached (double dist_tolerance, double angle_tolerance) const {
    (void) dist_tolerance;
    (void) angle_tolerance;
    return last_goal_reached_;
}

}  // namespace texnitis::nav_core::mppi
