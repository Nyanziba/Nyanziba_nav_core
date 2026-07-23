#pragma once

#include "texnitis_nav_core/types.hpp"

#include <span>
#include <vector>

namespace texnitis::nav_core {

struct MotionPrimitive {
    std::vector<Pose2D> samples;  // start-relative body frame; last is endpoint
    double              length{0.0};
    double              delta_yaw{0.0};
    bool                reverse{false};
    bool                in_place{false};
};

class MotionModel {
   public:
    virtual ~MotionModel ()                                                                                                       = default;
    [[nodiscard]] virtual bool                             valid () const noexcept                                                = 0;
    [[nodiscard]] virtual std::span<const MotionPrimitive> primitives () const noexcept                                           = 0;
    [[nodiscard]] virtual double                           transitionTime (const MotionPrimitive &primitive) const noexcept       = 0;
    [[nodiscard]] virtual double                           lowerBoundTime (const Pose2D &from, const Pose2D &goal) const noexcept = 0;
};

[[nodiscard]] double trapezoidalTravelTime (double distance, double max_velocity, double max_acceleration) noexcept;

}  // namespace texnitis::nav_core
