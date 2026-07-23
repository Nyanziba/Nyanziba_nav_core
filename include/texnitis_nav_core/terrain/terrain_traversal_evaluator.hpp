#pragma once

#include "texnitis_nav_core/types.hpp"

namespace texnitis::nav_core {

struct TerrainVehicleParams {
    double uphill_speed_scale_per_grade{1.5};
    double downhill_speed_scale_per_grade{0.25};
    double minimum_speed_scale{0.20};
    double maximum_longitudinal_grade{1.0};
    double track_width{0.50};
    double center_of_gravity_height{0.30};
    double rollover_safety_factor{0.80};
    bool   require_valid_slope{false};
};

struct TerrainTraversalContext {
    double travel_heading{0.0};
    double body_yaw{0.0};
    double gradient_x{0.0};
    double gradient_y{0.0};
};

struct TerrainTraversalEstimate {
    bool   traversable{true};
    double travel_time{0.0};
    double speed_scale{1.0};
    double longitudinal_grade{0.0};
    double lateral_grade{0.0};
    double rollover_margin{0.0};
};

class TerrainTraversalEvaluator {
   public:
    explicit TerrainTraversalEvaluator (TerrainVehicleParams params = {}) : params_ (params) {}
    [[nodiscard]] TerrainTraversalEstimate evaluate (double flat_travel_time, const TerrainTraversalContext &context) const noexcept;

   private:
    TerrainVehicleParams params_;
};

}  // namespace texnitis::nav_core
