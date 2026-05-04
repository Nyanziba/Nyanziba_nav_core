/// @file
/// @brief 8 近傍 A\* グリッドプランナー。
///
/// 旧 `texnitis_move_base_like::AStarPlanner` をベースに、ROS 依存を
/// 取り除いて純 C++ に書き直したもの。アルゴリズム本体（heuristic、
/// オープンセット運用、膨張処理）は同じだが、`OccupancyGrid` 直接参照
/// ではなく `GridMapView` を介する。`worldToMap` は `GridMapView`
/// のメンバに集約。
///
/// @see docs/algorithms/astar.md

#pragma once

#include "texnitis_nav_core/planner_base.hpp"
#include "texnitis_nav_core/planners/astar_params.hpp"

#include <atomic>
#include <cstdint>
#include <vector>

namespace texnitis::nav_core {

class AStarPlanner final : public PlannerBase {
   public:
    AStarPlanner () = default;
    explicit AStarPlanner (const AStarParams &params) : params_ (params) {}

    void setParams (const AStarParams &params) { params_ = params; }
    [[nodiscard]] const AStarParams &params () const noexcept { return params_; }

    [[nodiscard]] PlanResult planPath (const GridMapView &map,
                                       const Pose2D      &start,
                                       const Pose2D      &goal,
                                       Path2D            &out_path) override;

    void cancel () noexcept override { cancel_flag_.store (true, std::memory_order_relaxed); }

    void reset () override { cancel_flag_.store (false, std::memory_order_relaxed); }

   private:
    /// 膨張済みグリッド。`planPath` 内で `inflated_` に書き込む。
    /// メモリ確保を毎回繰り返さないようメンバに保持する。
    [[nodiscard]] bool inflate (const GridMapView &map);

    /// `inflated_` に対する 1 セル単位の lethal 判定。
    [[nodiscard]] bool isCellBlocked (int mx, int my) const noexcept;

    AStarParams params_{};

    std::vector<int8_t> inflated_;
    int                 inflated_width_{0};
    int                 inflated_height_{0};

    std::atomic<bool> cancel_flag_{false};
};

}  // namespace texnitis::nav_core
