#pragma once

#include "texnitis_nav_core/motion_models/motion_model.hpp"

namespace texnitis::nav_core {

struct DifferentialDriveMotionModelParams {
    double primitive_length{0.30};
    int    heading_bins{16};
    double max_linear_velocity{1.0};
    double max_linear_acceleration{1.0};
    double max_angular_velocity{1.5};
    double max_angular_acceleration{2.0};
    double minimum_turning_radius{0.30};
    bool   allow_in_place_rotation{true};
    bool   allow_reverse{false};
    double reverse_penalty{0.5};
};

class DifferentialDriveMotionModel final : public MotionModel {
   public:
    explicit DifferentialDriveMotionModel (DifferentialDriveMotionModelParams params = {});
    [[nodiscard]] bool                             valid () const noexcept override;
    [[nodiscard]] std::span<const MotionPrimitive> primitives () const noexcept override {
        return primitives_;
    }
    [[nodiscard]] double transitionTime (const MotionPrimitive &primitive) const noexcept override;
    [[nodiscard]] double lowerBoundTime (const Pose2D &from, const Pose2D &goal) const noexcept override;

   private:
    DifferentialDriveMotionModelParams params_;
    std::vector<MotionPrimitive>       primitives_;
};

}  // namespace texnitis::nav_core
