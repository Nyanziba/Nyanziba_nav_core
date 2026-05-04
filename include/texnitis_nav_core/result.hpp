/// @file
/// @brief PlanResult / ControllerResult の enum 定義と文字列化。
///
/// 旧 `bool` 戻り値では「失敗の理由」が握り潰されていた問題があり、
/// mbf アダプタ層の outcome 変換でも区別が必要なため、明示的な
/// enum で返す。
///
/// `texnitis_mbf_plugins/texnitis_mbf_common/error_codes.hpp` でこの
/// enum を mbf の `uint32_t outcome` (0=SUCCESS, 50=NO_PATH_FOUND, ...)
/// にマップする。

#pragma once

namespace texnitis::nav_core {

/// @brief プランナーの呼び出し結果。
///
/// 値は API 的に安定。新しい値は **末尾に追加** すること（既存値の
/// 数値順を変えない）。
enum class PlanResult {
    /// 経路が見つかった。
    Success = 0,

    /// 開始 pose がマップ範囲外。
    StartOutOfBounds,

    /// ゴール pose がマップ範囲外。
    GoalOutOfBounds,

    /// 開始 pose が衝突セル内。
    StartInCollision,

    /// ゴール pose が衝突セル内。
    GoalInCollision,

    /// 探索を完走したが経路が無かった。
    NoPathFound,

    /// マップが空 / data == nullptr 等。
    InvalidMap,

    /// `cancel()` が呼ばれて中断された。
    Cancelled,

    /// 必須リソース (例: HeightProvider 高さグリッド) が
    /// 未到着のまま planPath が呼ばれた。
    NotInitialized,

    /// 内部不変条件違反。これが返るのは実装バグ。
    InternalError
};

/// @brief コントローラの呼び出し結果。
enum class ControllerResult {
    Success = 0,

    /// ゴールに到達した（GoalChecker が True を返した）。`Success`
    /// と区別したいときに使う。区別不要なら `Success` で良い。
    GoalReached,

    /// 経路が空（または `setPlan` が呼ばれていない）。
    EmptyPath,

    /// 経路から大きく外れている等、追従不能と判断した。
    PathTooFarFromRobot,

    /// 解算 / シミュレーションが失敗した（NaN 検出など）。
    ComputationFailed,

    /// `cancel()` が呼ばれた。
    Cancelled,

    /// 必須パラメータ等が未設定。
    NotInitialized,

    /// 内部不変条件違反。
    InternalError
};

/// @brief 文字列化（ログ / アサーション用）。`switch` で全ケースを
///        書くので、enum 拡張時にコンパイラが警告してくれる。
[[nodiscard]] inline const char *toString (PlanResult result) noexcept {
    switch (result) {
        case PlanResult::Success:           return "Success";
        case PlanResult::StartOutOfBounds:  return "StartOutOfBounds";
        case PlanResult::GoalOutOfBounds:   return "GoalOutOfBounds";
        case PlanResult::StartInCollision:  return "StartInCollision";
        case PlanResult::GoalInCollision:   return "GoalInCollision";
        case PlanResult::NoPathFound:       return "NoPathFound";
        case PlanResult::InvalidMap:        return "InvalidMap";
        case PlanResult::Cancelled:         return "Cancelled";
        case PlanResult::NotInitialized:    return "NotInitialized";
        case PlanResult::InternalError:     return "InternalError";
    }
    return "Unknown";
}

[[nodiscard]] inline const char *toString (ControllerResult result) noexcept {
    switch (result) {
        case ControllerResult::Success:             return "Success";
        case ControllerResult::GoalReached:         return "GoalReached";
        case ControllerResult::EmptyPath:           return "EmptyPath";
        case ControllerResult::PathTooFarFromRobot: return "PathTooFarFromRobot";
        case ControllerResult::ComputationFailed:   return "ComputationFailed";
        case ControllerResult::Cancelled:           return "Cancelled";
        case ControllerResult::NotInitialized:      return "NotInitialized";
        case ControllerResult::InternalError:       return "InternalError";
    }
    return "Unknown";
}

}  // namespace texnitis::nav_core
