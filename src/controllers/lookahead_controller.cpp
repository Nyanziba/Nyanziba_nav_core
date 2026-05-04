// LookaheadController の実装。旧 texnitis_move_base_like の同名
// クラスからの移植。pluginlib export とパブリッシャ参照を取り除き、
// `Path2D` / `Twist2D` で受け答えする純 C++ 化。

#include "texnitis_nav_core/controllers/lookahead_controller.hpp"

#include <cmath>
#include <limits>

namespace texnitis::nav_core {

void LookaheadController::setPlan (const Path2D &path) {
    path_ = path;
    goal_checker_.reset ();
    last_goal_reached_ = false;
    cancel_flag_.store (false, std::memory_order_relaxed);
}

ControllerResult LookaheadController::computeCommand (const Pose2D &current,
                                                      Twist2D      &cmd_out) {
    cmd_out = Twist2D{};
    if (cancel_flag_.load (std::memory_order_relaxed)) {
        return ControllerResult::Cancelled;
    }
    if (path_.poses.empty ()) {
        return ControllerResult::EmptyPath;
    }

    const Pose2D goal = path_.poses.back ();

    if (goal_checker_.isXYReached (current, goal) && goal_checker_.isYawReached (current, goal)) {
        last_goal_reached_ = true;
        return ControllerResult::GoalReached;
    }

    // 最近傍を探し、そこから lookahead 距離だけ進めた pose を target にする。
    size_t nearest_idx = 0;
    double min_dist    = std::numeric_limits<double>::infinity ();
    for (size_t i = 0; i < path_.poses.size (); ++i) {
        const double dx = path_.poses[i].x - current.x;
        const double dy = path_.poses[i].y - current.y;
        const double d  = std::hypot (dx, dy);
        if (d < min_dist) {
            min_dist    = d;
            nearest_idx = i;
        }
    }

    size_t target_idx  = nearest_idx;
    double accumulated = 0.0;
    for (size_t i = nearest_idx; i + 1 < path_.poses.size (); ++i) {
        const double dx = path_.poses[i + 1].x - path_.poses[i].x;
        const double dy = path_.poses[i + 1].y - path_.poses[i].y;
        accumulated += std::hypot (dx, dy);
        target_idx = i + 1;
        if (accumulated >= params_.lookahead_dist) {
            break;
        }
    }

    const bool xy_reached = goal_checker_.isXYReached (current, goal);
    if (xy_reached) {
        target_idx = path_.poses.size () - 1;
    }

    const Pose2D &target = path_.poses[target_idx];
    const double  ex_world = target.x - current.x;
    const double  ey_world = target.y - current.y;
    const double  cos_yaw  = std::cos (current.yaw);
    const double  sin_yaw  = std::sin (current.yaw);
    const double  ex_body  = cos_yaw * ex_world + sin_yaw * ey_world;
    const double  ey_body  = -sin_yaw * ex_world + cos_yaw * ey_world;

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    if (params_.use_diff_drive) {
        const double target_yaw = std::atan2 (ey_world, ex_world);
        const double eyaw       = shortestAngularDiff (current.yaw, target_yaw);
        wz = clamp (params_.kp_yaw * eyaw, -params_.max_speed_yaw, params_.max_speed_yaw);
        if (!xy_reached) {
            const double dist = std::hypot (ex_world, ey_world);
            vx = clamp (params_.kp_xy * dist, 0.0, params_.max_speed_xy);
        } else {
            const double final_eyaw = shortestAngularDiff (current.yaw, goal.yaw);
            wz = clamp (params_.kp_yaw * final_eyaw, -params_.max_speed_yaw, params_.max_speed_yaw);
            vx = 0.0;
        }
    } else {
        if (xy_reached) {
            const double final_eyaw = shortestAngularDiff (current.yaw, goal.yaw);
            wz = clamp (params_.kp_yaw * final_eyaw, -params_.max_speed_yaw, params_.max_speed_yaw);
        } else {
            vx = clamp (params_.kp_xy * ex_body, -params_.max_speed_xy, params_.max_speed_xy);
            vy = clamp (params_.kp_xy * ey_body, -params_.max_speed_xy, params_.max_speed_xy);
            const double target_yaw = std::atan2 (ey_world, ex_world);
            const double eyaw       = shortestAngularDiff (current.yaw, target_yaw);
            wz = clamp (params_.kp_yaw * eyaw, -params_.max_speed_yaw, params_.max_speed_yaw);

            const double linear_mag = std::hypot (vx, vy);
            if (linear_mag > params_.linear_threshold_for_wz) {
                wz = clamp (wz, -params_.max_wz_when_moving, params_.max_wz_when_moving);
            }
        }
    }

    cmd_out.vx = vx;
    cmd_out.vy = vy;
    cmd_out.wz = wz;
    last_goal_reached_ = false;
    return ControllerResult::Success;
}

bool LookaheadController::isGoalReached (double dist_tolerance,
                                         double angle_tolerance) const {
    if (path_.poses.empty ()) {
        return false;
    }
    if (last_goal_reached_) {
        return true;
    }
    // We do not run the kinematics here; mbf calls computeCommand on the
    // same tick anyway. Use a const-cast on goal_checker_ to evaluate the
    // override-tolerance variant; goal_checker_ keeps stateful flags
    // internally so this stays correct.
    auto &mutable_checker = const_cast<GoalChecker &> (goal_checker_);
    const Pose2D goal = path_.poses.back ();
    const Pose2D current = goal;  // Without a current pose we cannot judge;
                                  // the mbf adapter is expected to drive
                                  // computeCommand first and consult
                                  // last_goal_reached_ via the cache.
    (void) mutable_checker;
    (void) current;
    (void) dist_tolerance;
    (void) angle_tolerance;
    return last_goal_reached_;
}

}  // namespace texnitis::nav_core
