/// @file
/// @brief 高さグリッドの最新値を提供する抽象。subscription はコア外。
///
/// HeightAwareAStarPlanner は ROS の `create_subscription` を直接持たず、
/// この抽象を通して「最新の HeightGridView」を借りに行く。実装は
/// mbf アダプタ側の `HeightMapProvider` または、ユニットテスト用の
/// `StubHeightProvider`。
///
/// 借用の寿命は呼び出し側が `planPath` の間だけ保証する想定（コアは
/// 中で `getLatest` を呼んだあと、抽象が指す内部バッファが planPath
/// 終了まで生存していることを期待する）。

#pragma once

#include "texnitis_nav_core/grid_map_view.hpp"

namespace texnitis::nav_core {

class HeightProvider {
   public:
    virtual ~HeightProvider () = default;

    /// @brief 最新の高さグリッドが取得できれば書き込んで true を返す。
    ///        まだ未到着なら false。
    [[nodiscard]] virtual bool getLatest (HeightGridView &out_view) const = 0;
};

}  // namespace texnitis::nav_core
