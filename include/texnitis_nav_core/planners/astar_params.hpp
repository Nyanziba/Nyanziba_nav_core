/// @file
/// @brief AStarPlanner のパラメータ。POD で構造体定義のみ。
///
/// ROS パラメータ（旧 `astar_planner.cpp` の `declare_parameter` 群）
/// を構造体に変換した形。mbf アダプタ層が ROS パラメータからこの
/// 構造体を埋めて `AStarPlanner` のコンストラクタに渡す。

#pragma once

namespace texnitis::nav_core {

struct AStarParams {
    /// 占有閾値 [0..100]。これ以上のセルは通行不可。
    int occupied_threshold{65};

    /// `-1`（unknown）セルを障害物扱いするか。
    bool unknown_is_obstacle{true};

    /// 膨張半径 [m]。0 で膨張なし。
    double inflation_radius{0.30};

    /// プランナーが許容する最大ステップ数（無限ループ防止）。
    /// 0 にすると幅×高さの 2 倍を自動採用。
    int max_iterations{0};

    /// ヒューリスティクスの重み。1.0 で admissible、>1.0 で速度優先。
    double heuristic_weight{1.0};

    /// 8 近傍探索を許可する。false なら 4 近傍のみ。
    bool allow_diagonal{true};
};

}  // namespace texnitis::nav_core
