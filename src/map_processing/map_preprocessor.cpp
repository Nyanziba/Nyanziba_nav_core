#include "texnitis_nav_core/map_processing/map_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace texnitis::nav_core {

ProcessedPlanningMap ProcessedPlanningMapStorage::view () const noexcept {
    ProcessedPlanningMap out;
    out.occupancy      = geometry_;
    out.occupancy.data = occupancy_.empty () ? nullptr : occupancy_.data ();
    out.traversal_cost = traversal_cost_.empty () ? nullptr : traversal_cost_.data ();
    out.elevation_m    = elevation_m_.empty () ? nullptr : elevation_m_.data ();
    out.gradient_x     = gradient_x_.empty () ? nullptr : gradient_x_.data ();
    out.gradient_y     = gradient_y_.empty () ? nullptr : gradient_y_.data ();
    return out;
}

PreprocessResult MapPreprocessor::process (const GridMapView &map, const TerrainLayersView &terrain, ProcessedPlanningMapStorage &out) const {
    const bool has_elevation = terrain.elevation && terrain.elevation->valid ();
    const bool has_slope     = terrain.slope && terrain.slope->valid ();
    const auto policy        = params_.missing_terrain_policy;
    if ((policy == MissingTerrainPolicy::RequireElevation && !has_elevation) || (policy == MissingTerrainPolicy::RequireSlope && !has_slope) || (policy == MissingTerrainPolicy::RequireAll && (!has_elevation || !has_slope))) {
        return PreprocessResult::MissingTerrain;
    }
    if (map.data == nullptr || map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
        return PreprocessResult::InvalidMap;
    }
    if (params_.occupied_threshold < 0 || params_.occupied_threshold > 100 || params_.inflation_radius < 0.0 || params_.slope_cost_scale < 0.0) {
        return PreprocessResult::InvalidParameter;
    }

    const size_t total = static_cast<size_t> (map.width) * map.height;
    out.geometry_      = map;
    out.geometry_.data = nullptr;
    out.occupancy_.assign (total, 0);
    out.traversal_cost_.assign (total, 0.0F);
    out.elevation_m_.assign (total, std::numeric_limits<float>::quiet_NaN ());
    out.gradient_x_.assign (total, std::numeric_limits<float>::quiet_NaN ());
    out.gradient_y_.assign (total, std::numeric_limits<float>::quiet_NaN ());

    for (size_t i = 0; i < total; ++i) {
        const int value   = map.data[i];
        out.occupancy_[i] = (value < 0 && params_.unknown_is_obstacle) || value >= params_.occupied_threshold ? 100 : 0;
    }

    const auto sample_elevation = [&] (double wx, double wy) {
        if (!has_elevation) return std::numeric_limits<float>::quiet_NaN ();
        int x = 0, y = 0;
        return terrain.elevation->worldToMap (wx, wy, x, y) ? terrain.elevation->at (x, y) : std::numeric_limits<float>::quiet_NaN ();
    };

    for (int y = 0; y < map.height; ++y) {
        for (int x = 0; x < map.width; ++x) {
            const size_t idx = static_cast<size_t> (y) * map.width + x;
            double       wx = 0.0, wy = 0.0;
            map.mapToWorld (x, y, wx, wy);
            const float elevation = sample_elevation (wx, wy);
            out.elevation_m_[idx] = elevation;

            float gx = NAN, gy = NAN;
            if (has_slope) {
                int sx = 0, sy = 0;
                if (terrain.slope->gradient_x.worldToMap (wx, wy, sx, sy)) {
                    gx = terrain.slope->gradient_x.at (sx, sy);
                    gy = terrain.slope->gradient_y.at (sx, sy);
                }
            } else if (has_elevation && std::isfinite (elevation)) {
                const float ex0 = sample_elevation (wx - map.resolution, wy);
                const float ex1 = sample_elevation (wx + map.resolution, wy);
                const float ey0 = sample_elevation (wx, wy - map.resolution);
                const float ey1 = sample_elevation (wx, wy + map.resolution);
                if (std::isfinite (ex0) && std::isfinite (ex1)) gx = (ex1 - ex0) / static_cast<float> (2.0 * map.resolution);
                if (std::isfinite (ey0) && std::isfinite (ey1)) gy = (ey1 - ey0) / static_cast<float> (2.0 * map.resolution);

                if (params_.max_step_height > 0.0) {
                    for (float adjacent : {ex0, ex1, ey0, ey1}) {
                        if (std::isfinite (adjacent) && std::abs (adjacent - elevation) > params_.max_step_height) {
                            out.occupancy_[idx] = 100;
                        }
                    }
                }
            }
            out.gradient_x_[idx] = gx;
            out.gradient_y_[idx] = gy;
            if (std::isfinite (gx) && std::isfinite (gy)) {
                const double angle = std::atan (std::hypot (gx, gy));
                if (params_.max_slope > 0.0 && angle > params_.max_slope)
                    out.occupancy_[idx] = 100;
                else
                    out.traversal_cost_[idx] = static_cast<float> (angle * params_.slope_cost_scale);
            }
        }
    }

    if (params_.inflation_radius > 0.0) {
        const int  radius    = static_cast<int> (std::ceil (params_.inflation_radius / map.resolution));
        const int  radius_sq = radius * radius;
        const auto source    = out.occupancy_;
        for (int y = 0; y < map.height; ++y)
            for (int x = 0; x < map.width; ++x) {
                if (source[static_cast<size_t> (y) * map.width + x] < 100) continue;
                for (int dy = -radius; dy <= radius; ++dy)
                    for (int dx = -radius; dx <= radius; ++dx) {
                        if (dx * dx + dy * dy > radius_sq) continue;
                        const int nx = x + dx, ny = y + dy;
                        if (map.inBounds (nx, ny)) out.occupancy_[static_cast<size_t> (ny) * map.width + nx] = 100;
                    }
            }
    }
    return PreprocessResult::Success;
}

}  // namespace texnitis::nav_core
