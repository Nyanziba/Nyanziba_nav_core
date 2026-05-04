// Minimal helper that wraps a real `rclcpp::Node` for use by the legacy
// texnitis_move_base_like plugins under test. We deliberately use a real node
// rather than a hand-rolled stub so plugin code paths
// (declare_parameter / get_parameter / get_logger / now / create_subscription)
// behave exactly as in production. The wrapper only exists so that:
//
//  - rclcpp::init / shutdown is owned in one place
//  - parameters can be loaded from a flat std::unordered_map<string,Value>
//    that the C ABI receives from Python
//  - subscriptions can be primed by injecting messages directly, bypassing
//    the executor (the harness drives the plugin synchronously, no spin)
//
// Used only by tests/legacy_harness; never linked into production code.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace texnitis::nav_core::legacy_harness {

using ParamValue = std::variant<bool, int64_t, double, std::string,
                                std::vector<double>>;

class FakeNode {
   public:
    /**
     * @brief Initialise rclcpp once per process.
     *
     * Idempotent: subsequent calls after the first init are no-ops.
     */
    static void initRclcpp ();

    /**
     * @brief Shut rclcpp down. Only the last live FakeNode should call this.
     */
    static void shutdownRclcpp ();

    /**
     * @brief Construct a real rclcpp::Node with parameter overrides applied.
     *
     * @param name     Node name. Must be unique per process.
     * @param params   Flat parameter map. Keys are dotted parameter names as
     *                 produced by SCHEMA.md (`pp_max_speed_xy` etc.).
     */
    FakeNode (const std::string &name,
              const std::unordered_map<std::string, ParamValue> &params);

    ~FakeNode ();

    rclcpp::Node::SharedPtr node () const { return node_; }

   private:
    rclcpp::Node::SharedPtr node_;
};

}  // namespace texnitis::nav_core::legacy_harness
