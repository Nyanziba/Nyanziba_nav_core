// extern "C" surface implementation. Skeleton: the structure / handle table /
// error reporting are in place, but the per-plugin glue (parameter loading
// from JSON, ROS message construction, planPath / computeCommand wiring)
// is left as TODO blocks for the next M0 commit. The intent here is to make
// the surface area and ownership rules concrete before filling in details.

#include "c_api.hpp"

#include "fake_node/fake_node.hpp"

#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <pluginlib/class_loader.hpp>
#include <texnitis_move_base_like/controller_base.hpp>
#include <texnitis_move_base_like/planner_base.hpp>

namespace tnh = texnitis::nav_core::legacy_harness;
namespace tmbl = texnitis_move_base_like;

namespace {

constexpr int32_t kStatusArgError       = -1;
constexpr int32_t kStatusInternalError  = -2;
constexpr int32_t kStatusNotInitialized = 59;

void writeError (char *buf, size_t buf_len, const std::string &message) {
    if (buf == nullptr || buf_len == 0) {
        return;
    }
    const size_t copy_n = std::min (buf_len - 1, message.size ());
    std::memcpy (buf, message.data (), copy_n);
    buf[copy_n] = '\0';
}

struct PlannerEntry {
    std::shared_ptr<tnh::FakeNode>                            host;
    std::shared_ptr<pluginlib::ClassLoader<tmbl::PlannerBase>> loader;
    std::shared_ptr<tmbl::PlannerBase>                         instance;
};

struct ControllerEntry {
    std::shared_ptr<tnh::FakeNode>                                host;
    std::shared_ptr<pluginlib::ClassLoader<tmbl::ControllerBase>> loader;
    std::shared_ptr<tmbl::ControllerBase>                          instance;
    std::vector<std::array<double, 3>>                              cached_path;
};

struct Registry {
    std::mutex                                       mutex;
    std::unordered_map<int32_t, PlannerEntry>        planners;
    std::unordered_map<int32_t, ControllerEntry>     controllers;
    int32_t                                          next_planner_handle{1};
    int32_t                                          next_controller_handle{1};
};

Registry &registry () {
    static Registry r;
    return r;
}

// TODO(M0): json -> std::unordered_map<std::string, ParamValue>
// implementation. We will pull in nlohmann_json (FetchContent) here and
// translate the flat JSON object that legacy_runner.py builds.
std::unordered_map<std::string, tnh::ParamValue>
parseParamJson (const char *param_json) {
    (void) param_json;
    return {};
}

}  // namespace

extern "C" {

int32_t legacy_init (void) {
    try {
        tnh::FakeNode::initRclcpp ();
        return 0;
    } catch (const std::exception &) {
        return kStatusInternalError;
    }
}

int32_t legacy_shutdown (void) {
    try {
        tnh::FakeNode::shutdownRclcpp ();
        return 0;
    } catch (const std::exception &) {
        return kStatusInternalError;
    }
}

int32_t legacy_planner_create (const char *plugin_name,
                               const char *param_json,
                               char       *err_buf,
                               size_t      err_buf_len) {
    if (plugin_name == nullptr) {
        writeError (err_buf, err_buf_len, "plugin_name is null");
        return kStatusArgError;
    }

    try {
        const auto params = parseParamJson (param_json);

        auto host = std::make_shared<tnh::FakeNode> (
            std::string ("legacy_planner_") + plugin_name, params);

        auto loader = std::make_shared<pluginlib::ClassLoader<tmbl::PlannerBase>> (
            "texnitis_move_base_like", "texnitis_move_base_like::PlannerBase");
        auto instance = loader->createSharedInstance (plugin_name);
        instance->initialize (host->node ());

        auto &reg = registry ();
        std::lock_guard<std::mutex> lock (reg.mutex);
        const int32_t handle = reg.next_planner_handle++;
        reg.planners.emplace (handle,
                              PlannerEntry{std::move (host),
                                           std::move (loader),
                                           std::move (instance)});
        return handle;
    } catch (const std::exception &ex) {
        writeError (err_buf, err_buf_len, ex.what ());
        return kStatusInternalError;
    }
}

int32_t legacy_planner_plan (int32_t,
                             const int8_t *,
                             int32_t,
                             int32_t,
                             double,
                             double,
                             double,
                             const double *,
                             const double *,
                             double *,
                             int32_t,
                             int32_t *,
                             char   *err_buf,
                             size_t  err_buf_len) {
    // TODO(M0): translate the inputs into nav_msgs::msg::OccupancyGrid +
    // Pose2D, call PlannerBase::planPath, and then emit each PoseStamped
    // into the caller's flat double buffer (3 doubles per pose).
    writeError (err_buf, err_buf_len, "legacy_planner_plan not implemented yet");
    return kStatusNotInitialized;
}

int32_t legacy_planner_cancel (int32_t) {
    return 0;  // legacy PlannerBase has no cancel() yet; no-op is safe.
}

int32_t legacy_planner_destroy (int32_t handle) {
    auto &reg = registry ();
    std::lock_guard<std::mutex> lock (reg.mutex);
    return reg.planners.erase (handle) ? 0 : kStatusArgError;
}

int32_t legacy_controller_create (const char *plugin_name,
                                  const char *param_json,
                                  char       *err_buf,
                                  size_t      err_buf_len) {
    if (plugin_name == nullptr) {
        writeError (err_buf, err_buf_len, "plugin_name is null");
        return kStatusArgError;
    }

    try {
        const auto params = parseParamJson (param_json);

        auto host = std::make_shared<tnh::FakeNode> (
            std::string ("legacy_controller_") + plugin_name, params);

        auto loader = std::make_shared<pluginlib::ClassLoader<tmbl::ControllerBase>> (
            "texnitis_move_base_like", "texnitis_move_base_like::ControllerBase");
        auto instance = loader->createSharedInstance (plugin_name);
        instance->initialize (host->node ());

        auto &reg = registry ();
        std::lock_guard<std::mutex> lock (reg.mutex);
        const int32_t handle = reg.next_controller_handle++;
        reg.controllers.emplace (handle,
                                 ControllerEntry{std::move (host),
                                                 std::move (loader),
                                                 std::move (instance),
                                                 {}});
        return handle;
    } catch (const std::exception &ex) {
        writeError (err_buf, err_buf_len, ex.what ());
        return kStatusInternalError;
    }
}

int32_t legacy_controller_set_path (int32_t,
                                    const double *,
                                    int32_t,
                                    char  *err_buf,
                                    size_t err_buf_len) {
    writeError (err_buf, err_buf_len, "legacy_controller_set_path not implemented yet");
    return kStatusNotInitialized;
}

int32_t legacy_controller_compute (int32_t,
                                   const double *,
                                   const double *,
                                   double       *,
                                   int32_t      *,
                                   char         *err_buf,
                                   size_t        err_buf_len) {
    // TODO(M0): convert (Path2D + Pose2D) into nav_msgs::msg::Path +
    // Pose2D as the legacy interface expects, call computeCommand, copy
    // the resulting Twist back to out_cmd.
    writeError (err_buf, err_buf_len, "legacy_controller_compute not implemented yet");
    return kStatusNotInitialized;
}

int32_t legacy_controller_reset (int32_t handle) {
    auto &reg = registry ();
    std::lock_guard<std::mutex> lock (reg.mutex);
    auto it = reg.controllers.find (handle);
    if (it == reg.controllers.end ()) {
        return kStatusArgError;
    }
    it->second.instance->reset ();
    it->second.cached_path.clear ();
    return 0;
}

int32_t legacy_controller_cancel (int32_t handle) {
    return legacy_controller_reset (handle);
}

int32_t legacy_controller_destroy (int32_t handle) {
    auto &reg = registry ();
    std::lock_guard<std::mutex> lock (reg.mutex);
    return reg.controllers.erase (handle) ? 0 : kStatusArgError;
}

int32_t legacy_planner_set_height_grid (int32_t,
                                        const int8_t *,
                                        int32_t,
                                        int32_t,
                                        double,
                                        double,
                                        double,
                                        char  *err_buf,
                                        size_t err_buf_len) {
    writeError (err_buf, err_buf_len, "legacy_planner_set_height_grid not implemented yet");
    return kStatusNotInitialized;
}

}  // extern "C"
