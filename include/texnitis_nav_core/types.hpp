/// @file
/// @brief 共通の POD 型と数値ユーティリティ。ここには `rclcpp` も
///        `geometry_msgs` も入れない。すべての高レイヤヘッダがこれを
///        最初に取り込むので、依存方向の最下層として振る舞う。
///
/// 読む順序: types.hpp → grid_map_view.hpp → result.hpp → logger.hpp
///           → planner_base.hpp / controller_base.hpp。
///
/// @see docs/reading_guide.md

#pragma once

#include <cmath>
#include <vector>

namespace texnitis::nav_core {

/// @brief π。`<numbers>` (C++20) も使えるが、ROS 1 / 旧コンパイラとの
///        互換性のために自前定数を残す。`std::numbers::pi_v<double>`
///        と等価。
inline constexpr double kPi = 3.14159265358979323846;

/// @brief 2D pose (x, y, yaw)。
///
/// 単位: x/y は [m]、yaw は [rad]。`yaw` は `[-π, π]` に正規化された
/// 状態を保つよう、生成側 (planner / runner) は `normalizeAngle` を
/// 通して書き込むこと。読み手が再正規化しても挙動は変わらないが、
/// API の不変条件として明示する。
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

/// @brief 2D twist (vx, vy, wz)。
///
/// 差動駆動では `vy` を必ず 0.0 にする。メカナム / ホロノミックでは
/// 全成分を使う。座標系は base_link 系（前方 +x、左 +y、上 +z 軸まわり
/// 反時計回りが正）。
struct Twist2D {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
};

/// @brief 計画されたパス。
///
/// `poses[0]` がスタート、`poses.back()` がゴール。各 pose の `yaw`
/// は接線方向ヒント（pure pursuit 用）として書き込まれることが多い
/// が、`yaw` の実装側からの信頼度はプランナーごとに異なる。
struct Path2D {
    std::vector<Pose2D> poses;
};

/// @brief 角度を `[-π, π]` に正規化。
///
/// `while` ループは入力の周回数が極端に多い場合に低速だが、ナビゲー
/// ションでは入力が `O(2π)` 程度に収まる前提なので問題にならない。
[[nodiscard]] inline constexpr double normalizeAngle (double angle) noexcept {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

/// @brief `value` を `[lo, hi]` にクランプ。
///
/// `std::clamp` と同じだが、`constexpr` で常に使える形にする
/// （`std::clamp` は C++17 で `constexpr` 化されているので実質同じ）。
[[nodiscard]] inline constexpr double clamp (double value, double lo, double hi) noexcept {
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

/// @brief 2 点間のユークリッド距離。`std::hypot` を使い double で返す。
[[nodiscard]] inline double distanceXY (const Pose2D &a, const Pose2D &b) noexcept {
    return std::hypot (b.x - a.x, b.y - a.y);
}

/// @brief 2 つの yaw の最短回転角差を `[-π, π]` で返す。
[[nodiscard]] inline double shortestAngularDiff (double from_yaw, double to_yaw) noexcept {
    return normalizeAngle (to_yaw - from_yaw);
}

}  // namespace texnitis::nav_core
