/// @file
/// @brief HeightAwareAStarPlanner のパラメータ。
///
/// AStarParams の sub-set + 高さグリッド固有のしきい値を持つ。
/// 詳細は docs/algorithms/height_aware_astar.md を参照。

#pragma once

#include "texnitis_nav_core/planners/astar_params.hpp"

namespace texnitis::nav_core {

struct HeightAwareAStarParams {
    /// 占有グリッドに関する設定（A* と共通）。
    AStarParams astar{};

    /// 高さ値（int8 [0,100] を高さの離散値として解釈）の lethal 閾値。
    /// これ以上のセルは通行不可とみなして占有グリッドにマージする。
    int height_lethal_threshold{50};

    /// 高さグリッドが未到着の状態で planPath が呼ばれたとき、
    /// `PlanResult::NotInitialized` で即座に失敗するか
    /// （false なら高さ情報を無視して通常の A* として走らせる）。
    bool require_height_grid{true};
};

}  // namespace texnitis::nav_core
