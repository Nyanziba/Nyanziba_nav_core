/// @file
/// @brief 占有グリッドの zero-copy ビュー型。
///
/// `nav_msgs::msg::OccupancyGrid` への直接依存を避けるため、データ部
/// (`int8_t*`)、寸法 (`width`, `height`)、解像度、原点だけを borrow
/// する。所有権はビュー側にない。寿命は呼び出し側 (mbf アダプタ /
/// シミュレータ) が `OccupancyGrid::data` の寿命を planPath 呼び出し
/// 中保証する。
///
/// 旧 texnitis_move_base_like では `worldToMap` を A* と
/// HeightAwareAStar が個別に実装していた。`GridMapView::worldToMap`
/// に集約してその重複を解消する。
///
/// @see docs/algorithms/astar.md, docs/architecture.md

#pragma once

#include "texnitis_nav_core/types.hpp"

#include <cstdint>

namespace texnitis::nav_core {

/// @brief 占有グリッドのビュー。値は `[-1, 100]` のいずれかで、
///        `-1` は unknown、`0..100` は占有確率パーセント。
///
/// `data == nullptr` の場合は「マップ未到着」状態。利用側は
/// `planPath` の冒頭で `inBounds(...) == false` ではなく
/// `data != nullptr` を確認してから使う。
struct GridMapView {
    const int8_t *data{nullptr};

    int    width{0};
    int    height{0};
    double resolution{0.05};
    double origin_x{0.0};
    double origin_y{0.0};

    /// @brief 世界座標 → グリッド index への変換。
    ///
    /// @param wx, wy 世界座標 [m]
    /// @param mx, my [out] グリッド index。範囲外でも書き込まれる
    ///        （呼び出し側でデバッグできるように）。
    /// @return `inBounds(mx, my)` のとき true。
    [[nodiscard]] bool worldToMap (double wx, double wy, int &mx, int &my) const noexcept {
        mx = static_cast<int> ((wx - origin_x) / resolution);
        my = static_cast<int> ((wy - origin_y) / resolution);
        return inBounds (mx, my);
    }

    /// @brief グリッド index → 世界座標。中心点を返す。
    void mapToWorld (int mx, int my, double &wx, double &wy) const noexcept {
        wx = origin_x + (static_cast<double> (mx) + 0.5) * resolution;
        wy = origin_y + (static_cast<double> (my) + 0.5) * resolution;
    }

    /// @brief 範囲内かどうか。`width / height` のどちらかが 0 でも
    ///        false を返す（empty な map）。
    [[nodiscard]] bool inBounds (int mx, int my) const noexcept {
        return mx >= 0 && my >= 0 && mx < width && my < height;
    }

    /// @brief 範囲内なら値を、範囲外なら `100` (= occupied) を返す。
    ///        `-1` (unknown) は呼び出し側で `unknown_is_obstacle`
    ///        フラグに従って扱う。
    [[nodiscard]] int8_t at (int mx, int my) const noexcept {
        if (data == nullptr || !inBounds (mx, my)) {
            return 100;
        }
        return data[my * width + mx];
    }

    /// @brief world 座標で直接 `at` を呼ぶ薄い糖衣。
    [[nodiscard]] int8_t atWorld (double wx, double wy) const noexcept {
        int mx = 0;
        int my = 0;
        if (!worldToMap (wx, wy, mx, my)) {
            return 100;
        }
        return at (mx, my);
    }
};

/// @brief 高さグリッドのビュー。`HeightAwareAStarPlanner` が
///        `HeightProvider` 経由で受け取る。`int8_t` を `[0, 100]`
///        の高さ（任意単位、プランナーが解釈）として扱う。
struct HeightGridView {
    const int8_t *data{nullptr};

    int    width{0};
    int    height{0};
    double resolution{0.05};
    double origin_x{0.0};
    double origin_y{0.0};

    [[nodiscard]] bool worldToMap (double wx, double wy, int &mx, int &my) const noexcept {
        mx = static_cast<int> ((wx - origin_x) / resolution);
        my = static_cast<int> ((wy - origin_y) / resolution);
        return inBounds (mx, my);
    }

    [[nodiscard]] bool inBounds (int mx, int my) const noexcept {
        return mx >= 0 && my >= 0 && mx < width && my < height;
    }

    [[nodiscard]] int8_t at (int mx, int my) const noexcept {
        if (data == nullptr || !inBounds (mx, my)) {
            return 0;
        }
        return data[my * width + mx];
    }
};

}  // namespace texnitis::nav_core
