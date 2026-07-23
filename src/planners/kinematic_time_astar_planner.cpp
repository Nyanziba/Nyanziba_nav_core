#include "texnitis_nav_core/planners/kinematic_time_astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace texnitis::nav_core {
namespace {
struct State {
    int x;
    int y;
    int heading;
};
struct OpenNode {
    State    state;
    double   f;
    uint64_t sequence;
};
struct OpenGreater {
    bool operator() (const OpenNode &a, const OpenNode &b) const noexcept {
        if (a.f != b.f) return a.f > b.f;
        return a.sequence > b.sequence;
    }
};
struct ParentEdge {
    uint64_t parent_key{0};
    double   duration{0.0};
};
uint64_t keyOf (const State &s, int width, int bins) noexcept {
    return (static_cast<uint64_t> (s.y) * width + s.x) * bins + s.heading;
}
State stateOf (uint64_t key, int width, int bins) noexcept {
    State s{};
    s.heading = static_cast<int> (key % bins);
    key /= bins;
    s.x = static_cast<int> (key % width);
    s.y = static_cast<int> (key / width);
    return s;
}
double headingToYaw (int heading, int bins) noexcept {
    return normalizeAngle (2.0 * kPi * heading / bins);
}
int yawToHeading (double yaw, int bins) noexcept {
    const double wrapped = normalizeAngle (yaw);
    int          value   = static_cast<int> (std::llround (wrapped * bins / (2.0 * kPi)));
    value %= bins;
    if (value < 0) value += bins;
    return value;
}
}  // namespace

KinematicTimeAStarPlanner::KinematicTimeAStarPlanner (KinematicTimeAStarParams params, std::shared_ptr<const MotionModel> model) : params_ (params), motion_model_ (std::move (model)), preprocessor_ (params.map_preprocessor), terrain_evaluator_ (params.terrain_vehicle) {}

PlanResult KinematicTimeAStarPlanner::planPath (const GridMapView &map, const Pose2D &start, const Pose2D &goal, Path2D &out_path) {
    return planPathWithTerrain (map, {}, start, goal, out_path);
}

PlanResult KinematicTimeAStarPlanner::planPathWithTerrain (const GridMapView &map, const TerrainLayersView &terrain, const Pose2D &start, const Pose2D &goal, Path2D &out_path) {
    Trajectory2D trajectory;
    const auto result = planTrajectoryWithTerrain (map, terrain, start, goal, trajectory);
    out_path.poses.clear ();
    out_path.poses.reserve (trajectory.points.size ());
    for (const auto &point : trajectory.points) out_path.poses.push_back (point.pose);
    return result;
}

PlanResult KinematicTimeAStarPlanner::planTrajectory (const GridMapView &map, const Pose2D &start, const Pose2D &goal, Trajectory2D &out_trajectory) {
    return planTrajectoryWithTerrain (map, {}, start, goal, out_trajectory);
}

PlanResult KinematicTimeAStarPlanner::planTrajectoryWithTerrain (const GridMapView &map, const TerrainLayersView &terrain, const Pose2D &start, const Pose2D &goal, Trajectory2D &out_trajectory) {
    out_trajectory.points.clear ();
    last_plan_cost_ = 0.0;
    cancel_flag_.store (false, std::memory_order_relaxed);
    if (!motion_model_ || !motion_model_->valid () || params_.heading_bins < 4 || params_.planning_timeout <= 0.0 || params_.heuristic_weight <= 0.0) return PlanResult::NotInitialized;

    const auto preprocess = preprocessor_.process (map, terrain, processed_storage_);
    if (preprocess == PreprocessResult::InvalidMap) return PlanResult::InvalidMap;
    if (preprocess == PreprocessResult::MissingTerrain) return PlanResult::NotInitialized;
    if (preprocess != PreprocessResult::Success) return PlanResult::InternalError;
    const auto  processed = processed_storage_.view ();
    const auto &grid      = processed.occupancy;

    int sx = 0, sy = 0, gx = 0, gy = 0;
    if (!grid.worldToMap (start.x, start.y, sx, sy)) return PlanResult::StartOutOfBounds;
    if (!grid.worldToMap (goal.x, goal.y, gx, gy)) return PlanResult::GoalOutOfBounds;
    if (grid.at (sx, sy) >= 100) return PlanResult::StartInCollision;
    if (grid.at (gx, gy) >= 100) return PlanResult::GoalInCollision;

    const State                                                       start_state{sx, sy, yawToHeading (start.yaw, params_.heading_bins)};
    std::priority_queue<OpenNode, std::vector<OpenNode>, OpenGreater> open;
    std::unordered_map<uint64_t, double>                              score;
    std::unordered_map<uint64_t, ParentEdge>                          parent;
    const uint64_t                                                    start_key = keyOf (start_state, grid.width, params_.heading_bins);
    score[start_key]                                                            = 0.0;
    open.push ({start_state, motion_model_->lowerBoundTime (start, goal), 0});
    uint64_t   sequence       = 1;
    uint64_t   reached_key    = 0;
    bool       reached        = false;
    const int  max_iterations = params_.max_iterations > 0 ? params_.max_iterations : grid.width * grid.height * params_.heading_bins * 2;
    const auto begin          = std::chrono::steady_clock::now ();

    for (int iterations = 0; !open.empty () && iterations < max_iterations; ++iterations) {
        if (cancel_flag_.load (std::memory_order_relaxed)) return PlanResult::Cancelled;
        if (std::chrono::duration<double> (std::chrono::steady_clock::now () - begin).count () > params_.planning_timeout) return PlanResult::NoPathFound;
        const OpenNode current_node = open.top ();
        open.pop ();
        const State    current     = current_node.state;
        const uint64_t current_key = keyOf (current, grid.width, params_.heading_bins);
        const double   current_g   = score[current_key];
        if (current.x == gx && current.y == gy && std::abs (shortestAngularDiff (headingToYaw (current.heading, params_.heading_bins), goal.yaw)) <= params_.goal_yaw_tolerance) {
            reached     = true;
            reached_key = current_key;
            break;
        }

        double cx = 0.0, cy = 0.0;
        grid.mapToWorld (current.x, current.y, cx, cy);
        const double cyaw = headingToYaw (current.heading, params_.heading_bins);
        for (const auto &primitive : motion_model_->primitives ()) {
            if (primitive.samples.empty ()) continue;
            bool collision = false;
            int  nx = current.x, ny = current.y;
            for (const auto &sample : primitive.samples) {
                const double wx = cx + std::cos (cyaw) * sample.x - std::sin (cyaw) * sample.y;
                const double wy = cy + std::sin (cyaw) * sample.x + std::cos (cyaw) * sample.y;
                if (!grid.worldToMap (wx, wy, nx, ny) || grid.at (nx, ny) >= 100) {
                    collision = true;
                    break;
                }
            }
            if (collision) continue;
            const int   nh = yawToHeading (cyaw + primitive.delta_yaw, params_.heading_bins);
            const State next{nx, ny, nh};
            if (next.x == current.x && next.y == current.y && next.heading == current.heading) continue;
            const uint64_t next_key   = keyOf (next, grid.width, params_.heading_bins);
            double         transition = motion_model_->transitionTime (primitive);
            if (!std::isfinite (transition)) continue;
            if (terrain.slope && terrain.slope->valid ()) {
                const float gx_value = terrain.slope->gradient_x.at (nx, ny);
                const float gy_value = terrain.slope->gradient_y.at (nx, ny);
                const auto &endpoint = primitive.samples.back ();
                const double travel_heading = primitive.in_place
                                                  ? cyaw
                                                  : normalizeAngle (cyaw + std::atan2 (endpoint.y, endpoint.x));
                const auto estimate = terrain_evaluator_.evaluate (
                    transition, {travel_heading, cyaw, gx_value, gy_value});
                if (!estimate.traversable) continue;
                transition = estimate.travel_time;
            }
            const size_t cell = static_cast<size_t> (ny) * grid.width + nx;
            if (processed.traversal_cost) transition += params_.traversal_cost_weight * processed.traversal_cost[cell];
            const double tentative = current_g + transition;
            auto         old       = score.find (next_key);
            if (old != score.end () && tentative >= old->second) continue;
            score[next_key]  = tentative;
            parent[next_key] = {current_key, transition};
            Pose2D next_pose;
            grid.mapToWorld (nx, ny, next_pose.x, next_pose.y);
            next_pose.yaw  = headingToYaw (nh, params_.heading_bins);
            const double h = motion_model_->lowerBoundTime (next_pose, goal);
            open.push ({next, tentative + params_.heuristic_weight * h, sequence++});
        }
    }
    if (!reached) return PlanResult::NoPathFound;

    std::vector<State>  reversed;
    std::vector<double> reversed_durations;
    for (uint64_t key = reached_key;; key = parent.at (key).parent_key) {
        reversed.push_back (stateOf (key, grid.width, params_.heading_bins));
        if (key == start_key) break;
        if (!parent.count (key)) return PlanResult::InternalError;
        reversed_durations.push_back (parent.at (key).duration);
    }
    out_trajectory.points.reserve (reversed.size ());
    for (auto it = reversed.rbegin (); it != reversed.rend (); ++it) {
        Pose2D pose;
        grid.mapToWorld (it->x, it->y, pose.x, pose.y);
        pose.yaw = headingToYaw (it->heading, params_.heading_bins);
        out_trajectory.points.push_back ({pose, {}, {}, 0.0});
    }
    out_trajectory.points.front ().pose = start;
    out_trajectory.points.back ().pose  = goal;
    std::reverse (reversed_durations.begin (), reversed_durations.end ());
    for (size_t i = 0; i < reversed_durations.size (); ++i) {
        const double dt = reversed_durations[i];
        auto &from = out_trajectory.points[i];
        auto &to   = out_trajectory.points[i + 1];
        to.time_from_start = from.time_from_start + dt;
        const double dx = to.pose.x - from.pose.x;
        const double dy = to.pose.y - from.pose.y;
        from.velocity.vx = (std::cos (from.pose.yaw) * dx + std::sin (from.pose.yaw) * dy) / dt;
        from.velocity.vy = (-std::sin (from.pose.yaw) * dx + std::cos (from.pose.yaw) * dy) / dt;
        from.velocity.wz = shortestAngularDiff (from.pose.yaw, to.pose.yaw) / dt;
    }
    for (size_t i = 1; i + 1 < out_trajectory.points.size (); ++i) {
        const double dt = out_trajectory.points[i].time_from_start - out_trajectory.points[i - 1].time_from_start;
        out_trajectory.points[i].acceleration.vx = (out_trajectory.points[i].velocity.vx - out_trajectory.points[i - 1].velocity.vx) / dt;
        out_trajectory.points[i].acceleration.vy = (out_trajectory.points[i].velocity.vy - out_trajectory.points[i - 1].velocity.vy) / dt;
        out_trajectory.points[i].acceleration.wz = (out_trajectory.points[i].velocity.wz - out_trajectory.points[i - 1].velocity.wz) / dt;
    }
    last_plan_cost_         = score[reached_key];
    return PlanResult::Success;
}

}  // namespace texnitis::nav_core
