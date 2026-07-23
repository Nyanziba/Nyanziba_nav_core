#pragma once

#include "texnitis_nav_core/grid_map_view.hpp"
#include "texnitis_nav_core/map_processing/terrain_grid.hpp"

#include <cstdint>
#include <vector>

namespace texnitis::nav_core {

enum class PreprocessResult {
    Success,
    InvalidMap,
    MissingTerrain,
    InvalidParameter
};

struct MapPreprocessorParams {
    int                  occupied_threshold{65};
    bool                 unknown_is_obstacle{true};
    double               inflation_radius{0.30};
    double               max_step_height{0.05};  // [m], <= 0 disables
    double               max_slope{0.35};        // [rad], <= 0 disables
    double               slope_cost_scale{1.0};
    MissingTerrainPolicy missing_terrain_policy{MissingTerrainPolicy::Allow2DOnly};
};

struct ProcessedPlanningMap {
    GridMapView  occupancy;
    const float *traversal_cost{nullptr};
    const float *elevation_m{nullptr};
    const float *gradient_x{nullptr};
    const float *gradient_y{nullptr};
};

class ProcessedPlanningMapStorage {
   public:
    [[nodiscard]] ProcessedPlanningMap view () const noexcept;

   private:
    friend class MapPreprocessor;
    GridMapView         geometry_{};
    std::vector<int8_t> occupancy_;
    std::vector<float>  traversal_cost_;
    std::vector<float>  elevation_m_;
    std::vector<float>  gradient_x_;
    std::vector<float>  gradient_y_;
};

class MapPreprocessor {
   public:
    MapPreprocessor () = default;
    explicit MapPreprocessor (MapPreprocessorParams params) : params_ (params) {}

    [[nodiscard]] PreprocessResult process (const GridMapView &occupancy, const TerrainLayersView &terrain, ProcessedPlanningMapStorage &output) const;

    [[nodiscard]] const MapPreprocessorParams &params () const noexcept {
        return params_;
    }

   private:
    MapPreprocessorParams params_{};
};

}  // namespace texnitis::nav_core
