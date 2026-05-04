// 8 近傍 A* の実装。旧 texnitis_move_base_like::AStarPlanner からの
// 移植。アルゴリズム本体は同等で、依存を rclcpp / nav_msgs から
// `GridMapView` / `Path2D` に置換し、パラメータは AStarParams から
// 引く。`worldToMap` は GridMapView::worldToMap に集約済み。

#include "texnitis_nav_core/planners/astar_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace texnitis::nav_core {

namespace {

/// 1 ノードのインデックスをパックして unordered_map / hash で扱える
/// よう 64bit にする。`my * width + mx` でも良いが、A* の探索範囲が
/// 巨大マップで int を超えうる場面を予防的に。
struct CellIndex {
    int x{0};
    int y{0};
};

inline int64_t cellKey (int x, int y) noexcept {
    return (static_cast<int64_t> (y) << 32) | static_cast<uint32_t> (x);
}

/// 8 近傍のオフセット (dx, dy, cost)。コストはセル間距離 [cells]。
constexpr std::array<std::array<double, 3>, 8> kNeighbours8{{
    {{ 1.0,  0.0, 1.0}},
    {{-1.0,  0.0, 1.0}},
    {{ 0.0,  1.0, 1.0}},
    {{ 0.0, -1.0, 1.0}},
    {{ 1.0,  1.0, 1.41421356}},
    {{-1.0,  1.0, 1.41421356}},
    {{ 1.0, -1.0, 1.41421356}},
    {{-1.0, -1.0, 1.41421356}},
}};

constexpr std::array<std::array<double, 3>, 4> kNeighbours4{{
    {{ 1.0,  0.0, 1.0}},
    {{-1.0,  0.0, 1.0}},
    {{ 0.0,  1.0, 1.0}},
    {{ 0.0, -1.0, 1.0}},
}};

/// Octile heuristic. 8 近傍 A* で admissible。
double octileDistance (int dx, int dy) noexcept {
    const int adx = std::abs (dx);
    const int ady = std::abs (dy);
    const int min_d = std::min (adx, ady);
    const int max_d = std::max (adx, ady);
    return (max_d - min_d) + std::sqrt (2.0) * static_cast<double> (min_d);
}

}  // namespace

bool AStarPlanner::inflate (const GridMapView &map) {
    inflated_width_  = map.width;
    inflated_height_ = map.height;
    const size_t total = static_cast<size_t> (map.width) * static_cast<size_t> (map.height);
    inflated_.assign (total, 0);

    if (map.data == nullptr || total == 0) {
        return false;
    }

    // 元データを取り込む（unknown=-1 を扱うので int8 のまま保持）。
    for (size_t i = 0; i < total; ++i) {
        inflated_[i] = map.data[i];
    }

    if (params_.inflation_radius <= 0.0 || map.resolution <= 0.0) {
        return true;
    }

    const int radius_cells =
        static_cast<int> (std::ceil (params_.inflation_radius / map.resolution));
    if (radius_cells <= 0) {
        return true;
    }
    const int radius_sq = radius_cells * radius_cells;

    std::vector<int8_t> source = inflated_;
    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            const int8_t value = source[static_cast<size_t> (y) * map.width + x];
            if (value < params_.occupied_threshold) {
                continue;
            }
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    if (dx * dx + dy * dy > radius_sq) {
                        continue;
                    }
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) {
                        continue;
                    }
                    const size_t idx = static_cast<size_t> (ny) * map.width + nx;
                    if (inflated_[idx] < 100) {
                        inflated_[idx] = 100;
                    }
                }
            }
        }
    }
    return true;
}

bool AStarPlanner::isCellBlocked (int mx, int my) const noexcept {
    if (mx < 0 || my < 0 || mx >= inflated_width_ || my >= inflated_height_) {
        return true;
    }
    const int8_t value = inflated_[static_cast<size_t> (my) * inflated_width_ + mx];
    if (value < 0) {
        return params_.unknown_is_obstacle;
    }
    return value >= params_.occupied_threshold;
}

PlanResult AStarPlanner::planPath (const GridMapView &map,
                                   const Pose2D      &start,
                                   const Pose2D      &goal,
                                   Path2D            &out_path) {
    out_path.poses.clear ();
    cancel_flag_.store (false, std::memory_order_relaxed);

    if (map.data == nullptr || map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
        log (LogLevel::Warn, "AStarPlanner: invalid map");
        return PlanResult::InvalidMap;
    }
    if (!inflate (map)) {
        return PlanResult::InvalidMap;
    }

    int start_x = 0;
    int start_y = 0;
    int goal_x  = 0;
    int goal_y  = 0;
    if (!map.worldToMap (start.x, start.y, start_x, start_y)) {
        log (LogLevel::Warn, "AStarPlanner: start out of bounds");
        return PlanResult::StartOutOfBounds;
    }
    if (!map.worldToMap (goal.x, goal.y, goal_x, goal_y)) {
        log (LogLevel::Warn, "AStarPlanner: goal out of bounds");
        return PlanResult::GoalOutOfBounds;
    }
    if (isCellBlocked (start_x, start_y)) {
        log (LogLevel::Warn, "AStarPlanner: start in collision");
        return PlanResult::StartInCollision;
    }
    if (isCellBlocked (goal_x, goal_y)) {
        log (LogLevel::Warn, "AStarPlanner: goal in collision");
        return PlanResult::GoalInCollision;
    }

    struct Node {
        int    x{0};
        int    y{0};
        double f{0.0};
    };
    struct GreaterByF {
        bool operator() (const Node &a, const Node &b) const noexcept { return a.f > b.f; }
    };

    std::priority_queue<Node, std::vector<Node>, GreaterByF> open;
    std::unordered_map<int64_t, double>  g_score;
    std::unordered_map<int64_t, int64_t> came_from;

    g_score[cellKey (start_x, start_y)] = 0.0;
    open.push (Node{start_x, start_y, octileDistance (goal_x - start_x, goal_y - start_y)});

    const int max_iter = params_.max_iterations > 0
                             ? params_.max_iterations
                             : 2 * map.width * map.height;
    int iters = 0;

    bool reached = false;

    while (!open.empty ()) {
        if (cancel_flag_.load (std::memory_order_relaxed)) {
            return PlanResult::Cancelled;
        }
        if (iters++ >= max_iter) {
            log (LogLevel::Warn, "AStarPlanner: max iterations exceeded");
            return PlanResult::NoPathFound;
        }
        const Node current = open.top ();
        open.pop ();

        if (current.x == goal_x && current.y == goal_y) {
            reached = true;
            break;
        }

        const auto step = [&] (const std::array<double, 3> &nb) {
            const int    nx   = current.x + static_cast<int> (nb[0]);
            const int    ny   = current.y + static_cast<int> (nb[1]);
            const double cost = nb[2];
            if (isCellBlocked (nx, ny)) {
                return;
            }
            const double tentative = g_score[cellKey (current.x, current.y)] + cost;
            const int64_t key = cellKey (nx, ny);
            auto it = g_score.find (key);
            if (it == g_score.end () || tentative < it->second) {
                g_score[key]   = tentative;
                came_from[key] = cellKey (current.x, current.y);
                const double h = octileDistance (goal_x - nx, goal_y - ny);
                open.push (Node{nx, ny, tentative + params_.heuristic_weight * h});
            }
        };

        if (params_.allow_diagonal) {
            for (const auto &nb : kNeighbours8) {
                step (nb);
            }
        } else {
            for (const auto &nb : kNeighbours4) {
                step (nb);
            }
        }
    }

    if (!reached) {
        log (LogLevel::Warn, "AStarPlanner: no path found");
        return PlanResult::NoPathFound;
    }

    // 経路を逆向きに辿って組み立てる。
    std::vector<std::pair<int, int>> reversed;
    int64_t key = cellKey (goal_x, goal_y);
    reversed.emplace_back (goal_x, goal_y);
    while (key != cellKey (start_x, start_y)) {
        auto it = came_from.find (key);
        if (it == came_from.end ()) {
            return PlanResult::InternalError;
        }
        key = it->second;
        const int x = static_cast<int> (key & 0xFFFFFFFF);
        const int y = static_cast<int> (key >> 32);
        reversed.emplace_back (x, y);
    }

    out_path.poses.reserve (reversed.size ());
    for (auto it = reversed.rbegin (); it != reversed.rend (); ++it) {
        Pose2D pose;
        map.mapToWorld (it->first, it->second, pose.x, pose.y);
        pose.yaw = 0.0;  // yaw は controller 側で接線方向から再計算する。
        out_path.poses.push_back (pose);
    }
    if (out_path.poses.empty ()) {
        return PlanResult::InternalError;
    }
    out_path.poses.back ().yaw = goal.yaw;
    return PlanResult::Success;
}

}  // namespace texnitis::nav_core
