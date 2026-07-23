#include "texnitis_nav_core/motion_models/motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace texnitis::nav_core {

double trapezoidalTravelTime (double distance, double velocity, double acceleration) noexcept {
    distance = std::abs (distance);
    if (distance == 0.0) return 0.0;
    if (velocity <= 0.0 || acceleration <= 0.0) return std::numeric_limits<double>::infinity ();
    const double ramp_distance = velocity * velocity / acceleration;
    if (distance <= ramp_distance) return 2.0 * std::sqrt (distance / acceleration);
    return 2.0 * velocity / acceleration + (distance - ramp_distance) / velocity;
}

}  // namespace texnitis::nav_core
