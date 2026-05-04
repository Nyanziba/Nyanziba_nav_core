// DiffDrivePurePursuitController: speed-scaled lookahead + 加速度上限
// + stateful GoalChecker。旧 texnitis_move_base_like の同名クラスから
// pluginlib エクスポートと publisher を取り除いた移植。

#include "texnitis_nav_core/controllers/diff_drive_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace texnitis::nav_core {

void DiffDrivePurePursuitController::setPlan (const Path2D &path) {
    path_ = path;
    goal_checker_.reset ();
    last_vx_ = 0.0;
    last_wz_ = 0.0;
    last_goal_reached_ = false;
    cancel_flag_.store (false, std::memory_order_relaxed);
}

size_t DiffDrivePurePursuitController::findClosestIndex (const Pose2D &current) const {
    size_t best_idx  = 0;
    double best_dist = std::numeric_limits<double>::infinity ();
    for (size_t i = 0; i < path_.poses.size (); ++i) {
        const double dx = path_.poses[i].x - current.x;
        const double dy = path_.poses[i].y - current.y;
        const double d  = std::hypot (dx, dy);
        if (d < best_dist) {
            best_dist = d;
            best_idx  = i;
        }
    }
    return best_idx;
}

size_t DiffDrivePurePursuitController::findLookaheadIndex (size_t closest_idx,
                                                            double lookahead_dist) const {
    if (path_.poses.empty ()) {
        return 0;
    }
    double accumulated = 0.0;
    size_t target_idx  = closest_idx;
    for (size_t i = closest_idx; i + 1 < path_.poses.size (); ++i) {
        const double dx = path_.poses[i + 1].x - path_.poses[i].x;
        const double dy = path_.poses[i + 1].y - path_.poses[i].y;
        accumulated += std::hypot (dx, dy);
        target_idx = i + 1;
        if (accumulated >= lookahead_dist) {
            break;
        }
    }
    return target_idx;
}

ControllerResult DiffDrivePurePursuitController::computeCommand (const Pose2D &current,
                                                                  Twist2D      &cmd_out) {
    cmd_out = Twist2D{};
    if (cancel_flag_.load (std::memory_order_relaxed)) {
        return ControllerResult::Cancelled;
    }
    if (path_.poses.empty ()) {
        return ControllerResult::EmptyPath;
    }

    const Pose2D goal = path_.poses.back ();
    if (goal_checker_.isReached (current, goal)) {
        last_goal_reached_ = true;
        last_vx_ = 0.0;
        last_wz_ = 0.0;
        return ControllerResult::GoalReached;
    }

    // Dynamic lookahead distance based on the previous linear velocity
    // (a proxy for the current speed setpoint).
    const double lookahead_dist = clamp (
        std::fabs (last_vx_) * params_.lookahead_time,
        params_.min_lookahead_distance,
        params_.max_lookahead_distance);

    const size_t closest_idx = findClosestIndex (current);
    const size_t target_idx  = findLookaheadIndex (closest_idx, lookahead_dist);

    const Pose2D &target = path_.poses[target_idx];
    const double  dx     = target.x - current.x;
    const double  dy     = target.y - current.y;
    const double  target_yaw = std::atan2 (dy, dx);
    const double  yaw_err    = shortestAngularDiff (current.yaw, target_yaw);

    // wz: proportional to yaw error, clamped, accel-limited.
    double wz_des = clamp (params_.angle_speed_p * yaw_err / kPi * params_.max_angular_velocity,
                           -params_.max_angular_velocity, params_.max_angular_velocity);
    const double wz_step = params_.max_angular_acceleration * params_.control_dt;
    double wz = clamp (wz_des, last_wz_ - wz_step, last_wz_ + wz_step);

    const bool xy_reached = goal_checker_.isXYReached (current, goal);
    double vx_des = 0.0;
    if (xy_reached) {
        // Final yaw alignment: stop translating.
        vx_des = 0.0;
        // Substitute a yaw command targeting the goal yaw rather than
        // the lookahead tangent.
        const double final_yaw_err = shortestAngularDiff (current.yaw, goal.yaw);
        wz_des = clamp (params_.angle_speed_p * final_yaw_err / kPi * params_.max_angular_velocity,
                        -params_.max_angular_velocity, params_.max_angular_velocity);
        wz = clamp (wz_des, last_wz_ - wz_step, last_wz_ + wz_step);
    } else {
        const double dist = std::hypot (dx, dy);
        vx_des = clamp (params_.max_linear_velocity * std::max (
                            0.0,
                            1.0 - params_.angle_speed_p * std::fabs (yaw_err) / kPi),
                        params_.min_linear_velocity, params_.max_linear_velocity);
        vx_des = std::min (vx_des, dist);
    }
    const double vx_step = params_.max_acceleration * params_.control_dt;
    const double vx = clamp (vx_des, last_vx_ - vx_step, last_vx_ + vx_step);

    cmd_out.vx = vx;
    cmd_out.vy = 0.0;
    cmd_out.wz = wz;
    last_vx_   = vx;
    last_wz_   = wz;
    last_goal_reached_ = false;
    return ControllerResult::Success;
}

bool DiffDrivePurePursuitController::isGoalReached (double dist_tolerance,
                                                    double angle_tolerance) const {
    (void) dist_tolerance;
    (void) angle_tolerance;
    return last_goal_reached_;
}

}  // namespace texnitis::nav_core
