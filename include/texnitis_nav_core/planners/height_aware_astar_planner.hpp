/// @file
/// @brief 高さグリッドを考慮した A* プランナー。
///
/// 実装方針: 旧 `texnitis_move_base_like::HeightAwareAStarPlanner` の
/// heading-bin 拡張は M9 段階で再導入する。M4 では「占有グリッド +
/// 高さしきい値超えセルを lethal にマージした合成マップ」に対して
/// 通常の `AStarPlanner` を走らせるシンプル版。これでも shape-of-API
/// は本実装と同じで、HeightProvider 抽象とのやり取りも検証できる。
///
/// @see docs/algorithms/height_aware_astar.md

#pragma once

#include "texnitis_nav_core/height_provider.hpp"
#include "texnitis_nav_core/planner_base.hpp"
#include "texnitis_nav_core/planners/astar_planner.hpp"
#include "texnitis_nav_core/planners/height_aware_astar_params.hpp"

#include <atomic>
#include <cstdint>
#include <vector>

namespace texnitis::nav_core {

class HeightAwareAStarPlanner final : public PlannerBase {
   public:
    HeightAwareAStarPlanner () = default;
    explicit HeightAwareAStarPlanner (const HeightAwareAStarParams &params,
                                       const HeightProvider          *provider = nullptr)
        : params_ (params), provider_ (provider), inner_ (params.astar) {}

    void setParams (const HeightAwareAStarParams &params) {
        params_ = params;
        inner_.setParams (params.astar);
    }

    void setHeightProvider (const HeightProvider *provider) noexcept { provider_ = provider; }

    [[nodiscard]] const HeightAwareAStarParams &params () const noexcept { return params_; }

    [[nodiscard]] PlanResult planPath (const GridMapView &map,
                                       const Pose2D      &start,
                                       const Pose2D      &goal,
                                       Path2D            &out_path) override;

    void cancel () noexcept override {
        cancel_flag_.store (true, std::memory_order_relaxed);
        inner_.cancel ();
    }

    void reset () override {
        cancel_flag_.store (false, std::memory_order_relaxed);
        inner_.reset ();
    }

    void setLogger (LoggerFn logger) override {
        PlannerBase::setLogger (logger);
        inner_.setLogger (std::move (logger));
    }

   private:
    HeightAwareAStarParams params_{};
    const HeightProvider  *provider_{nullptr};

    /// 占有グリッド + 高さしきい値超えセルをマージしたバッファ。
    /// `planPath` 中だけ生存する。
    std::vector<int8_t> merged_;

    AStarPlanner inner_;

    std::atomic<bool> cancel_flag_{false};
};

}  // namespace texnitis::nav_core
