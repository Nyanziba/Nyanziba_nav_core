// HeightAwareAStarPlanner: AStarPlanner にラップして「高さしきい値
// 超えセルを lethal にマージした合成マップ」を渡す実装。旧コードに
// あった heading-bin / yaw smoothing は M9 段階で再導入する想定。

#include "texnitis_nav_core/planners/height_aware_astar_planner.hpp"

#include <algorithm>
#include <cstring>

namespace texnitis::nav_core {

PlanResult HeightAwareAStarPlanner::planPath (const GridMapView &map,
                                              const Pose2D      &start,
                                              const Pose2D      &goal,
                                              Path2D            &out_path) {
    out_path.poses.clear ();
    cancel_flag_.store (false, std::memory_order_relaxed);

    HeightGridView height_view{};
    const bool height_available = provider_ ? provider_->getLatest (height_view) : false;
    if (!height_available && params_.require_height_grid) {
        log (LogLevel::Warn, "HeightAwareAStarPlanner: height grid not initialised");
        return PlanResult::NotInitialized;
    }

    if (map.data == nullptr || map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
        return PlanResult::InvalidMap;
    }

    // Build a merged occupancy view: copy original data, then mark
    // cells whose height value is at or above the threshold as lethal.
    const size_t total = static_cast<size_t> (map.width) * map.height;
    merged_.resize (total);
    std::memcpy (merged_.data (), map.data, total);

    if (height_available && height_view.data != nullptr) {
        for (int y = 0; y < map.height; ++y) {
            for (int x = 0; x < map.width; ++x) {
                double wx = 0.0;
                double wy = 0.0;
                map.mapToWorld (x, y, wx, wy);
                int hx = 0;
                int hy = 0;
                if (!height_view.worldToMap (wx, wy, hx, hy)) {
                    continue;
                }
                const int8_t h = height_view.at (hx, hy);
                if (h >= params_.height_lethal_threshold) {
                    merged_[static_cast<size_t> (y) * map.width + x] = 100;
                }
            }
        }
    }

    GridMapView merged_view = map;
    merged_view.data = merged_.data ();
    return inner_.planPath (merged_view, start, goal, out_path);
}

}  // namespace texnitis::nav_core
