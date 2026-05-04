#include "fake_node.hpp"

#include <atomic>
#include <mutex>
#include <stdexcept>

namespace texnitis::nav_core::legacy_harness {

namespace {

std::atomic<int> g_init_refcount{0};
std::mutex       g_init_mutex;

rclcpp::ParameterValue toRcl (const ParamValue &value) {
    return std::visit (
        [] (const auto &v) {
            return rclcpp::ParameterValue (v);
        },
        value);
}

}  // namespace

void FakeNode::initRclcpp () {
    std::lock_guard<std::mutex> guard (g_init_mutex);
    if (g_init_refcount.fetch_add (1) == 0) {
        if (!rclcpp::ok ()) {
            int    argc = 0;
            char **argv = nullptr;
            rclcpp::init (argc, argv);
        }
    }
}

void FakeNode::shutdownRclcpp () {
    std::lock_guard<std::mutex> guard (g_init_mutex);
    if (g_init_refcount.fetch_sub (1) == 1) {
        if (rclcpp::ok ()) {
            rclcpp::shutdown ();
        }
    }
}

FakeNode::FakeNode (const std::string &name,
                    const std::unordered_map<std::string, ParamValue> &params) {
    initRclcpp ();

    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides;
    overrides.reserve (params.size ());
    for (const auto &kv : params) {
        overrides.emplace_back (kv.first, toRcl (kv.second));
    }
    options.parameter_overrides (std::move (overrides));
    options.allow_undeclared_parameters (true);
    options.automatically_declare_parameters_from_overrides (true);

    node_ = std::make_shared<rclcpp::Node> (name, options);
}

FakeNode::~FakeNode () {
    node_.reset ();
    shutdownRclcpp ();
}

}  // namespace texnitis::nav_core::legacy_harness
