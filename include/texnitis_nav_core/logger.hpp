/// @file
/// @brief ログ抽象。`rclcpp::Logger` への依存をコアから外す。
///
/// 利用パターン:
///
/// ```cpp
/// auto planner = texnitis::nav_core::AStarPlanner(params);
/// planner.setLogger(texnitis::nav_core::defaultStderrLogger());
/// ```
///
/// mbf アダプタ側からは `rclcpp::Logger` をキャプチャしたラムダを
/// `setLogger` に渡し、コアから ROS のログマクロへ橋渡しする。
///
/// `std::ostream` は thread-safe ではなく、`std::println` (C++23) は
/// 環境差が大きいため、`std::function` 注入を採用する。
///
/// @see docs/design_rationale.md の「ロガー注入」節

#pragma once

#include <cstdio>
#include <functional>
#include <string_view>

namespace texnitis::nav_core {

/// @brief ログレベル。値は安定。RclLogger の typical な enum と
///        互換のある順序を保つ。
enum class LogLevel : int {
    Debug = 0,
    Info  = 1,
    Warn  = 2,
    Error = 3,
};

/// @brief ロガー。`nullptr` 同等のデフォルト構築値は「捨てる」動作。
using LoggerFn = std::function<void (LogLevel, std::string_view)>;

/// @brief LogLevel の文字列化。
[[nodiscard]] inline const char *toString (LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

/// @brief stderr に書き出すデフォルトロガー。テスト・サンプル用。
///
/// 本番（mbf プラグイン経由）では rclcpp::Logger に橋渡しするので、
/// このロガーを使うのは examples/ と単体テストのみ。
[[nodiscard]] inline LoggerFn defaultStderrLogger () {
    return [] (LogLevel level, std::string_view message) {
        std::fprintf (stderr,
                      "[texnitis_nav_core][%s] %.*s\n",
                      toString (level),
                      static_cast<int> (message.size ()),
                      message.data ());
    };
}

/// @brief ロガーが設定されていれば呼ぶ薄いユーティリティ。
inline void log (const LoggerFn &logger, LogLevel level, std::string_view message) {
    if (logger) {
        logger (level, message);
    }
}

}  // namespace texnitis::nav_core
