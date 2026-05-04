// demo: A* で経路を計画し、LookaheadController で追従する。
//
// 実行すると `trajectory.ppm` (Portable Pixmap, ASCII P3) を生成する。
// 占有グリッド (グレー)、計画経路 (青)、実走行軌跡 (緑)、開始位置 (緑)、
// 終了位置 (赤) を 1 枚に重ねた画像。Preview.app や ImageMagick で
// そのまま開ける。
//
// ビルド:
//   cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/install
//   cmake --build build -j
// 実行:
//   ./build/astar_lookahead

#include <texnitis_nav_core/controllers/lookahead_controller.hpp>
#include <texnitis_nav_core/grid_map_view.hpp>
#include <texnitis_nav_core/planners/astar_planner.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

namespace tnc = texnitis::nav_core;

namespace {

constexpr int    kWidth      = 50;
constexpr int    kHeight     = 30;
constexpr double kResolution = 0.10;          // [m / cell]
constexpr double kDt         = 0.05;          // [s]
constexpr int    kMaxSteps   = 600;

/// "コの字" の障害物を含むテストマップを作る。
std::vector<int8_t> makeMap () {
    std::vector<int8_t> grid (static_cast<size_t> (kWidth) * kHeight, 0);
    auto block = [&] (int x, int y) {
        if (x < 0 || y < 0 || x >= kWidth || y >= kHeight) return;
        grid[static_cast<size_t> (y) * kWidth + x] = 100;
    };
    for (int x = 10; x < 40; ++x) block (x, 12);
    for (int y = 12; y < 22; ++y) block (10, y);
    for (int y = 12; y < 22; ++y) block (39, y);
    return grid;
}

/// 1 ステップの unicycle 積分（vy は無視、yaw 正規化込み）。
void stepUnicycle (tnc::Pose2D &pose, const tnc::Twist2D &cmd, double dt) {
    pose.x += cmd.vx * std::cos (pose.yaw) * dt;
    pose.y += cmd.vx * std::sin (pose.yaw) * dt;
    pose.yaw = tnc::normalizeAngle (pose.yaw + cmd.wz * dt);
}

/// world 座標を整数ピクセル (col, row) に丸める。row は上下反転（PPM は左上原点）。
bool worldToPixel (double wx, double wy, int &col, int &row) {
    col = static_cast<int> (wx / kResolution);
    row = (kHeight - 1) - static_cast<int> (wy / kResolution);
    return col >= 0 && col < kWidth && row >= 0 && row < kHeight;
}

/// PPM (P3 ASCII) でグリッド + 経路 + 軌跡を保存する。
void savePpm (const std::string                              &path,
              const std::vector<int8_t>                       &grid,
              const tnc::Path2D                               &plan,
              const std::vector<tnc::Pose2D>                  &trajectory) {
    constexpr int kScale  = 12;          // 1 cell = 12 px
    const int     img_w   = kWidth * kScale;
    const int     img_h   = kHeight * kScale;

    std::vector<uint8_t> rgb (static_cast<size_t> (img_w) * img_h * 3, 240);
    auto put = [&] (int px, int py, uint8_t r, uint8_t g, uint8_t b) {
        if (px < 0 || py < 0 || px >= img_w || py >= img_h) return;
        const size_t base = (static_cast<size_t> (py) * img_w + px) * 3;
        rgb[base + 0] = r;
        rgb[base + 1] = g;
        rgb[base + 2] = b;
    };

    // 占有グリッドを濃淡で背景に。
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const int8_t v = grid[static_cast<size_t> (y) * kWidth + x];
            const uint8_t shade = (v >= 65) ? 30 : 245;
            for (int dy = 0; dy < kScale; ++dy) {
                for (int dx = 0; dx < kScale; ++dx) {
                    put (x * kScale + dx, (kHeight - 1 - y) * kScale + dy,
                         shade, shade, shade);
                }
            }
        }
    }

    // 計画経路を青で。
    for (const auto &p : plan.poses) {
        int col = 0, row = 0;
        if (!worldToPixel (p.x, p.y, col, row)) continue;
        const int px = col * kScale + kScale / 2;
        const int py = row * kScale + kScale / 2;
        put (px, py, 30, 60, 220);
        put (px - 1, py, 30, 60, 220);
        put (px + 1, py, 30, 60, 220);
    }

    // 実走行軌跡を緑で。
    for (size_t i = 0; i < trajectory.size (); ++i) {
        int col = 0, row = 0;
        if (!worldToPixel (trajectory[i].x, trajectory[i].y, col, row)) continue;
        const int px = col * kScale + kScale / 2;
        const int py = row * kScale + kScale / 2;
        put (px, py, 0, 160, 40);
    }

    // 開始 (緑) と終了 (赤) を強調。
    auto stamp = [&] (const tnc::Pose2D &pose, uint8_t r, uint8_t g, uint8_t b) {
        int col = 0, row = 0;
        if (!worldToPixel (pose.x, pose.y, col, row)) return;
        const int cx = col * kScale + kScale / 2;
        const int cy = row * kScale + kScale / 2;
        for (int dy = -3; dy <= 3; ++dy)
            for (int dx = -3; dx <= 3; ++dx)
                if (dx * dx + dy * dy <= 9) put (cx + dx, cy + dy, r, g, b);
    };
    if (!trajectory.empty ()) stamp (trajectory.front (), 0, 200, 60);
    if (!trajectory.empty ()) stamp (trajectory.back (), 220, 50, 50);

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
    const auto grid = makeMap ();

    tnc::GridMapView map;
    map.data       = grid.data ();
    map.width      = kWidth;
    map.height     = kHeight;
    map.resolution = kResolution;

    tnc::AStarParams aparams;
    aparams.inflation_radius = 0.20;
    tnc::AStarPlanner planner (aparams);

    const tnc::Pose2D start{0.5, 0.5, 0.0};
    const tnc::Pose2D goal {4.5, 2.5, 0.0};
    tnc::Path2D plan;
    if (planner.planPath (map, start, goal, plan) != tnc::PlanResult::Success) {
        std::cerr << "planner failed\n";
        return 1;
    }
    std::cout << "planned " << plan.poses.size () << " poses\n";

    tnc::LookaheadParams cparams;
    cparams.use_diff_drive   = false;
    cparams.lookahead_dist   = 0.40;
    cparams.max_speed_xy     = 0.40;
    cparams.max_speed_yaw    = 1.20;
    cparams.goal_checker.xy_tolerance  = 0.10;
    cparams.goal_checker.yaw_tolerance = 0.30;
    cparams.goal_checker.stateful      = true;
    tnc::LookaheadController controller (cparams);
    controller.setPlan (plan);

    std::vector<tnc::Pose2D> trajectory;
    trajectory.reserve (kMaxSteps);
    tnc::Pose2D pose = start;
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
        // 簡易メカナム積分（Lookahead はホロノミックを返すので vy も使う）。
        pose.x += (std::cos (pose.yaw) * cmd.vx - std::sin (pose.yaw) * cmd.vy) * kDt;
        pose.y += (std::sin (pose.yaw) * cmd.vx + std::cos (pose.yaw) * cmd.vy) * kDt;
        pose.yaw = tnc::normalizeAngle (pose.yaw + cmd.wz * kDt);
    }

    savePpm ("trajectory.ppm", grid, plan, trajectory);
    std::cout << "wrote trajectory.ppm (" << trajectory.size () << " steps)\n";
    return 0;
}
