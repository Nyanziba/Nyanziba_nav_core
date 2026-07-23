#include "texnitis_nav_core/terrain/terrain_traversal_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace texnitis::nav_core {

TerrainTraversalEstimate TerrainTraversalEvaluator::evaluate (double flat_time, const TerrainTraversalContext &context) const noexcept {
    TerrainTraversalEstimate out;
    out.travel_time = flat_time;
    if (!std::isfinite (context.gradient_x) || !std::isfinite (context.gradient_y)) {
        out.traversable = !params_.require_valid_slope;
        if (!out.traversable) out.travel_time = std::numeric_limits<double>::infinity ();
        return out;
    }

    out.longitudinal_grade = context.gradient_x * std::cos (context.travel_heading) + context.gradient_y * std::sin (context.travel_heading);
    out.lateral_grade      = -context.gradient_x * std::sin (context.body_yaw) + context.gradient_y * std::cos (context.body_yaw);
    const double half_track = 0.5 * params_.track_width;
    const double allowed_lateral_grade = params_.center_of_gravity_height > 0.0
                                             ? params_.rollover_safety_factor * half_track / params_.center_of_gravity_height
                                             : 0.0;
    out.rollover_margin = allowed_lateral_grade - std::abs (out.lateral_grade);
    if (std::abs (out.longitudinal_grade) > params_.maximum_longitudinal_grade || out.rollover_margin < 0.0) {
        out.traversable = false;
        out.travel_time = std::numeric_limits<double>::infinity ();
        return out;
    }

    const double penalty = out.longitudinal_grade >= 0.0
                               ? params_.uphill_speed_scale_per_grade * out.longitudinal_grade
                               : params_.downhill_speed_scale_per_grade * std::abs (out.longitudinal_grade);
    out.speed_scale = std::max (params_.minimum_speed_scale, 1.0 / (1.0 + penalty));
    out.travel_time = flat_time / out.speed_scale;
    return out;
}

}  // namespace texnitis::nav_core
