// demo: MPPI でメカナム車を経路追従させる。
//
// 直線パスを与え、サンプルベース MPPI が cmd を出して走行する様子を
// `mppi_trajectory.ppm` に書き出す。MPPI は Eigen のみで動くので、
// Eigen3 が見えていればこのデモはビルドできる。

#include <texnitis_nav_core/grid_map_view.hpp>
#include <texnitis_nav_core/mppi/mecanum_mppi_controller.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

namespace tnc = texnitis::nav_core;
namespace tnm = texnitis::nav_core::mppi;

namespace {

constexpr int    kWidth      = 60;
constexpr int    kHeight     = 24;
constexpr double kResolution = 0.10;
constexpr double kDt         = 0.05;
constexpr int    kMaxSteps   = 400;

bool worldToPixel (double wx, double wy, int &col, int &row) {
    col = static_cast<int> (wx / kResolution);
    row = (kHeight - 1) - static_cast<int> (wy / kResolution);
    return col >= 0 && col < kWidth && row >= 0 && row < kHeight;
}

void savePpm (const std::string                              &path,
              const std::vector<int8_t>                       &grid,
              const tnc::Path2D                               &plan,
              const std::vector<tnc::Pose2D>                  &trajectory) {
    constexpr int kScale = 12;
    const int     img_w  = kWidth * kScale;
    const int     img_h  = kHeight * kScale;
    std::vector<uint8_t> rgb (static_cast<size_t> (img_w) * img_h * 3, 240);
    auto put = [&] (int px, int py, uint8_t r, uint8_t g, uint8_t b) {
        if (px < 0 || py < 0 || px >= img_w || py >= img_h) return;
        const size_t base = (static_cast<size_t> (py) * img_w + px) * 3;
        rgb[base]     = r;
        rgb[base + 1] = g;
        rgb[base + 2] = b;
    };
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const uint8_t shade = (grid[static_cast<size_t> (y) * kWidth + x] >= 65) ? 30 : 245;
            for (int dy = 0; dy < kScale; ++dy)
                for (int dx = 0; dx < kScale; ++dx)
                    put (x * kScale + dx, (kHeight - 1 - y) * kScale + dy, shade, shade, shade);
        }
    }
    for (const auto &p : plan.poses) {
        int col = 0, row = 0;
        if (!worldToPixel (p.x, p.y, col, row)) continue;
        put (col * kScale + kScale / 2, row * kScale + kScale / 2, 30, 60, 220);
    }
    for (const auto &p : trajectory) {
        int col = 0, row = 0;
        if (!worldToPixel (p.x, p.y, col, row)) continue;
        put (col * kScale + kScale / 2, row * kScale + kScale / 2, 0, 160, 40);
    }
    std::ofstream out (path);
    out << "P3\n" << img_w << ' ' << img_h << "\n255\n";
    for (size_t i = 0; i < rgb.size (); i += 3) {
        out << static_cast<int> (rgb[i]) << ' '
            << static_cast<int> (rgb[i + 1]) << ' '
            << static_cast<int> (rgb[i + 2]) << '\n';
    }
}

}  // namespace

int main () {
    std::vector<int8_t> grid (static_cast<size_t> (kWidth) * kHeight, 0);
    tnc::GridMapView map;
    map.data       = grid.data ();
    map.width      = kWidth;
    map.height     = kHeight;
    map.resolution = kResolution;

    // 直線の経路 (0.5, 0.5) -> (5.5, 1.5) を 0.10 m 刻みで生成。
    tnc::Path2D plan;
    for (int i = 0; i <= 50; ++i) {
        const double t = static_cast<double> (i) / 50.0;
        plan.poses.push_back ({0.5 + 5.0 * t, 0.5 + 1.0 * t, 0.0});
    }

    tnm::MppiParams params;
    params.horizon                    = 20;
    params.num_samples                = 256;
    params.lambda                     = 0.10;
    params.dt                         = kDt;
    params.v_max                      = 0.50;
    params.omega_max                  = 1.50;
    params.goal_checker.xy_tolerance  = 0.10;
    params.goal_checker.yaw_tolerance = 0.30;
    params.goal_checker.stateful      = true;
    tnm::MecanumMppiController controller (params);
    controller.setPlan (plan);

    std::vector<tnc::Pose2D> trajectory;
    tnc::Pose2D pose{0.5, 0.5, 0.0};
    for (int step = 0; step < kMaxSteps; ++step) {
        tnc::Twist2D cmd;
        const auto status = controller.computeCommand (pose, cmd);
        trajectory.push_back (pose);
        if (status == tnc::ControllerResult::GoalReached) {
            std::cout << "goal reached at step " << step << '\n';
            break;
        }
        if (status != tnc::ControllerResult::Success) {
            std::cerr << "controller failed: " << tnc::toString (status) << '\n';
            break;
        }
        pose.x += (std::cos (pose.yaw) * cmd.vx - std::sin (pose.yaw) * cmd.vy) * kDt;
        pose.y += (std::sin (pose.yaw) * cmd.vx + std::cos (pose.yaw) * cmd.vy) * kDt;
        pose.yaw = tnc::normalizeAngle (pose.yaw + cmd.wz * kDt);

        // 占有衝突は無いマップだが、念のため `is_blocked` 相当のチェックを書ける場所として残す。
        (void) map;
    }

    savePpm ("mppi_trajectory.ppm", grid, plan, trajectory);
    std::cout << "wrote mppi_trajectory.ppm (" << trajectory.size () << " steps)\n";
    return 0;
}
