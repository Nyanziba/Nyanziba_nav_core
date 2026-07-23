#pragma once

#include "texnitis_nav_core/motion_models/motion_model.hpp"

namespace texnitis::nav_core {

struct HolonomicMotionModelParams {
    double primitive_length{0.30};
    int    heading_bins{16};
    double max_velocity_x{1.0};
    double max_velocity_y{1.0};
    double max_acceleration_x{1.0};
    double max_acceleration_y{1.0};
    double max_angular_velocity{1.5};
    double max_angular_acceleration{2.0};
    bool   allow_translation_while_rotating{true};
};

class HolonomicMotionModel final : public MotionModel {
   public:
    explicit HolonomicMotionModel (HolonomicMotionModelParams params = {});
    [[nodiscard]] bool                             valid () const noexcept override;
    [[nodiscard]] std::span<const MotionPrimitive> primitives () const noexcept override {
        return primitives_;
    }
    [[nodiscard]] double transitionTime (const MotionPrimitive &primitive) const noexcept override;
    [[nodiscard]] double lowerBoundTime (const Pose2D &from, const Pose2D &goal) const noexcept override;

   private:
    HolonomicMotionModelParams   params_;
    std::vector<MotionPrimitive> primitives_;
};

}  // namespace texnitis::nav_core
