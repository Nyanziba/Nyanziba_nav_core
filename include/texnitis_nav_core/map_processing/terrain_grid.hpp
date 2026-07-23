#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace texnitis::nav_core {

struct FloatGridView {
    const float   *data{nullptr};
    int            width{0};
    int            height{0};
    double         resolution{0.05};
    double         origin_x{0.0};
    double         origin_y{0.0};
    const uint8_t *valid_mask{nullptr};
    uint8_t        required_valid_bits{0};

    [[nodiscard]] bool valid () const noexcept {
        return data != nullptr && width > 0 && height > 0 && resolution > 0.0;
    }
    [[nodiscard]] bool inBounds (int x, int y) const noexcept {
        return x >= 0 && y >= 0 && x < width && y < height;
    }
    [[nodiscard]] bool worldToMap (double wx, double wy, int &x, int &y) const noexcept {
        x = static_cast<int> (std::floor ((wx - origin_x) / resolution));
        y = static_cast<int> (std::floor ((wy - origin_y) / resolution));
        return inBounds (x, y);
    }
    [[nodiscard]] float at (int x, int y) const noexcept {
        if (!valid () || !inBounds (x, y)) return NAN;
        const size_t index = static_cast<size_t> (y) * width + x;
        if (valid_mask && required_valid_bits != 0 && (valid_mask[index] & required_valid_bits) != required_valid_bits) return NAN;
        return data[index];
    }
};

/// Elevation values in metres. NaN denotes an unobserved cell.
using ElevationGridView = FloatGridView;

/// Surface gradient dz/dx and dz/dy (dimensionless rise/run).
struct SlopeGridView {
    FloatGridView gradient_x;
    FloatGridView gradient_y;

    [[nodiscard]] bool valid () const noexcept {
        return gradient_x.valid () && gradient_y.valid () && gradient_x.width == gradient_y.width && gradient_x.height == gradient_y.height && gradient_x.resolution == gradient_y.resolution && gradient_x.origin_x == gradient_y.origin_x &&
               gradient_x.origin_y == gradient_y.origin_y;
    }
};

struct TerrainLayersView {
    std::optional<ElevationGridView> elevation;
    std::optional<SlopeGridView>     slope;
};

enum class MissingTerrainPolicy {
    Allow2DOnly,
    RequireElevation,
    RequireSlope,
    RequireAll
};

}  // namespace texnitis::nav_core
