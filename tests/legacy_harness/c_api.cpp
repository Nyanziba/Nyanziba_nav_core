// extern "C" surface implementation. Drives the legacy
// texnitis_move_base_like planner / controller plugins from a Python
// process via ctypes. See c_api.hpp for the contract.
//
// This file is only built when ROS 2 Jazzy is available (see CMakeLists.txt
// guard). It will not compile in a ROS-less macOS dev shell; use
// `source ~/.pixi/envs/default/setup.zsh` first.

#include "c_api.hpp"

#include "fake_node/fake_node.hpp"

#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <texnitis_move_base_like/controller_base.hpp>
#include <texnitis_move_base_like/planner_base.hpp>

namespace tnh = texnitis::nav_core::legacy_harness;
namespace tmbl = texnitis_move_base_like;
using json = nlohmann::json;

namespace {

constexpr int32_t kStatusArgError       = -1;
constexpr int32_t kStatusInternalError  = -2;
constexpr int32_t kStatusNotInitialized = 59;

constexpr const char *kGlobalFrame = "map";

void writeError (char *buf, size_t buf_len, const std::string &message) {
    if (buf == nullptr || buf_len == 0) {
        return;
    }
    const size_t copy_n = std::min (buf_len - 1, message.size ());
    std::memcpy (buf, message.data (), copy_n);
    buf[copy_n] = '\0';
}

/// JSON object → ROS parameter overrides. Only handles the value kinds
/// that scenario.yaml produces today: bool / integer / float / string and
/// flat arrays of doubles. Anything else is silently dropped (and the
/// underlying parameter falls back to its plugin default).
std::unordered_map<std::string, tnh::ParamValue>
parseParamJson (const char *param_json) {
    std::unordered_map<std::string, tnh::ParamValue> out;
    if (param_json == nullptr || *param_json == '\0') {
        return out;
    }
    json doc = json::parse (param_json, nullptr, /*allow_exceptions=*/false);
    if (!doc.is_object ()) {
        return out;
    }
    for (auto it = doc.begin (); it != doc.end (); ++it) {
        const auto &value = it.value ();
        if (value.is_boolean ()) {
            out.emplace (it.key (), tnh::ParamValue{value.get<bool> ()});
        } else if (value.is_number_integer ()) {
            out.emplace (it.key (),
                         tnh::ParamValue{static_cast<int64_t> (value.get<int64_t> ())});
        } else if (value.is_number ()) {
            out.emplace (it.key (), tnh::ParamValue{value.get<double> ()});
        } else if (value.is_string ()) {
            out.emplace (it.key (), tnh::ParamValue{value.get<std::string> ()});
        } else if (value.is_array ()) {
            std::vector<double> arr;
            arr.reserve (value.size ());
            bool all_numeric = true;
            for (const auto &element : value) {
                if (!element.is_number ()) {
                    all_numeric = false;
                    break;
                }
                arr.push_back (element.get<double> ());
            }
            if (all_numeric) {
                out.emplace (it.key (), tnh::ParamValue{std::move (arr)});
            }
        }
    }
    return out;
}

/// Pack a flat (x, y, yaw) triple into a `geometry_msgs::msg::PoseStamped`.
geometry_msgs::msg::PoseStamped makePoseStamped (const double *xyz_yaw,
                                                 const std::string &frame_id,
                                                 const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = xyz_yaw[0];
    pose.pose.position.y = xyz_yaw[1];
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, xyz_yaw[2]);
    pose.pose.orientation.x = q.x ();
    pose.pose.orientation.y = q.y ();
    pose.pose.orientation.z = q.z ();
    pose.pose.orientation.w = q.w ();
    return pose;
}

double yawFromQuat (const geometry_msgs::msg::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2 (siny_cosp, cosy_cosp);
}

struct PlannerEntry {
    std::shared_ptr<tnh::FakeNode>                            host;
    std::shared_ptr<pluginlib::ClassLoader<tmbl::PlannerBase>> loader;
    std::shared_ptr<tmbl::PlannerBase>                         instance;
    nav_msgs::msg::OccupancyGrid                               cached_height_grid;
    bool                                                       has_height_grid{false};
};

struct ControllerEntry {
    std::shared_ptr<tnh::FakeNode>                                host;
    std::shared_ptr<pluginlib::ClassLoader<tmbl::ControllerBase>> loader;
    std::shared_ptr<tmbl::ControllerBase>                          instance;
    nav_msgs::msg::Path                                            cached_path;
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
                                           std::move (instance),
                                           {},
                                           false});
        return handle;
    } catch (const std::exception &ex) {
        writeError (err_buf, err_buf_len, ex.what ());
        return kStatusInternalError;
    }
}

int32_t legacy_planner_plan (int32_t       handle,
                             const int8_t *map_data,
                             int32_t       width,
                             int32_t       height,
                             double        resolution,
                             double        origin_x,
                             double        origin_y,
                             const double *start_pose,
                             const double *goal_pose,
                             double       *out_path,
                             int32_t       out_path_capacity,
                             int32_t      *out_path_len,
                             char         *err_buf,
                             size_t        err_buf_len) {
    if (map_data == nullptr || start_pose == nullptr || goal_pose == nullptr ||
        out_path == nullptr || out_path_len == nullptr || width <= 0 || height <= 0) {
        writeError (err_buf, err_buf_len, "invalid argument(s)");
        return kStatusArgError;
    }

    try {
        auto &reg = registry ();
        std::shared_ptr<tmbl::PlannerBase> planner;
        std::shared_ptr<tnh::FakeNode>     host;
        {
            std::lock_guard<std::mutex> lock (reg.mutex);
            auto it = reg.planners.find (handle);
            if (it == reg.planners.end ()) {
                writeError (err_buf, err_buf_len, "unknown planner handle");
                return kStatusArgError;
            }
            planner = it->second.instance;
            host    = it->second.host;
        }

        nav_msgs::msg::OccupancyGrid map;
        map.header.frame_id   = kGlobalFrame;
        map.header.stamp      = host->node ()->now ();
        map.info.resolution   = static_cast<float> (resolution);
        map.info.width        = static_cast<uint32_t> (width);
        map.info.height       = static_cast<uint32_t> (height);
        map.info.origin.position.x = origin_x;
        map.info.origin.position.y = origin_y;
        map.info.origin.orientation.w = 1.0;
        map.data.assign (map_data, map_data + (static_cast<size_t> (width) * height));

        texnitis_move_base_like::Pose2D start{start_pose[0], start_pose[1], start_pose[2]};
        texnitis_move_base_like::Pose2D goal{goal_pose[0], goal_pose[1], goal_pose[2]};

        nav_msgs::msg::Path planned;
        const bool ok = planner->planPath (map, start, goal, kGlobalFrame, planned);
        if (!ok) {
            *out_path_len = 0;
            return 50;  // NO_PATH_FOUND
        }

        const int32_t n_poses = std::min<int32_t> (
            out_path_capacity, static_cast<int32_t> (planned.poses.size ()));
        for (int32_t i = 0; i < n_poses; ++i) {
            const auto &p = planned.poses[i].pose;
            out_path[i * 3 + 0] = p.position.x;
            out_path[i * 3 + 1] = p.position.y;
            out_path[i * 3 + 2] = yawFromQuat (p.orientation);
        }
        *out_path_len = n_poses;
        return 0;  // SUCCESS
    } catch (const std::exception &ex) {
        writeError (err_buf, err_buf_len, ex.what ());
        return kStatusInternalError;
    }
}

int32_t legacy_planner_cancel (int32_t) {
    return 0;  // legacy PlannerBase has no cancel(); no-op is safe.
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

int32_t legacy_controller_set_path (int32_t       handle,
                                    const double *path,
                                    int32_t       path_len,
                                    char         *err_buf,
                                    size_t        err_buf_len) {
    if (path == nullptr || path_len < 0) {
        writeError (err_buf, err_buf_len, "invalid path argument");
        return kStatusArgError;
    }

    auto &reg = registry ();
    std::lock_guard<std::mutex> lock (reg.mutex);
    auto it = reg.controllers.find (handle);
    if (it == reg.controllers.end ()) {
        writeError (err_buf, err_buf_len, "unknown controller handle");
        return kStatusArgError;
    }

    it->second.cached_path.poses.clear ();
    it->second.cached_path.header.frame_id = kGlobalFrame;
    it->second.cached_path.header.stamp = it->second.host->node ()->now ();
    it->second.cached_path.poses.reserve (static_cast<size_t> (path_len));
    for (int32_t i = 0; i < path_len; ++i) {
        const double *p = path + i * 3;
        it->second.cached_path.poses.push_back (
            makePoseStamped (p, kGlobalFrame, it->second.cached_path.header.stamp));
    }
    it->second.instance->reset ();
    return 0;
}

int32_t legacy_controller_compute (int32_t       handle,
                                   const double *current_pose,
                                   const double *current_vel,
                                   double       *out_cmd,
                                   int32_t      *out_goal_reached,
                                   char         *err_buf,
                                   size_t        err_buf_len) {
    (void) current_vel;  // legacy ControllerBase does not consume velocity.
    if (current_pose == nullptr || out_cmd == nullptr || out_goal_reached == nullptr) {
        writeError (err_buf, err_buf_len, "invalid argument(s)");
        return kStatusArgError;
    }

    try {
        auto &reg = registry ();
        std::lock_guard<std::mutex> lock (reg.mutex);
        auto it = reg.controllers.find (handle);
        if (it == reg.controllers.end ()) {
            writeError (err_buf, err_buf_len, "unknown controller handle");
            return kStatusArgError;
        }

        if (it->second.cached_path.poses.empty ()) {
            out_cmd[0] = out_cmd[1] = out_cmd[2] = 0.0;
            *out_goal_reached = 0;
            return 104;  // INVALID_PATH (matches our nav_core::ControllerResult mapping)
        }

        texnitis_move_base_like::Pose2D pose{current_pose[0], current_pose[1], current_pose[2]};
        geometry_msgs::msg::Twist cmd;
        bool goal_reached = false;
        const bool ok = it->second.instance->computeCommand (
            pose, it->second.cached_path, kGlobalFrame, cmd, goal_reached);
        if (!ok) {
            out_cmd[0] = out_cmd[1] = out_cmd[2] = 0.0;
            *out_goal_reached = 0;
            return 100;  // COMPUTATION_FAILED
        }
        out_cmd[0] = cmd.linear.x;
        out_cmd[1] = cmd.linear.y;
        out_cmd[2] = cmd.angular.z;
        *out_goal_reached = goal_reached ? 1 : 0;
        return 0;
    } catch (const std::exception &ex) {
        writeError (err_buf, err_buf_len, ex.what ());
        return kStatusInternalError;
    }
}

int32_t legacy_controller_reset (int32_t handle) {
    auto &reg = registry ();
    std::lock_guard<std::mutex> lock (reg.mutex);
    auto it = reg.controllers.find (handle);
    if (it == reg.controllers.end ()) {
        return kStatusArgError;
    }
    it->second.instance->reset ();
    it->second.cached_path.poses.clear ();
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

int32_t legacy_planner_set_height_grid (int32_t       handle,
                                        const int8_t *height_data,
                                        int32_t       width,
                                        int32_t       height,
                                        double        resolution,
                                        double        origin_x,
                                        double        origin_y,
                                        char         *err_buf,
                                        size_t        err_buf_len) {
    // The legacy HeightAwareAStar planner consumes its height grid via a
    // ROS subscription. We cannot inject the grid synchronously without
    // patching the legacy code. Instead, the harness publishes onto the
    // expected topic and lets the FakeNode's executor deliver it.
    if (height_data == nullptr || width <= 0 || height <= 0) {
        writeError (err_buf, err_buf_len, "invalid argument(s)");
        return kStatusArgError;
    }

    try {
        auto &reg = registry ();
        std::shared_ptr<tnh::FakeNode> host;
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.frame_id        = kGlobalFrame;
        msg.info.resolution        = static_cast<float> (resolution);
        msg.info.width             = static_cast<uint32_t> (width);
        msg.info.height            = static_cast<uint32_t> (height);
        msg.info.origin.position.x = origin_x;
        msg.info.origin.position.y = origin_y;
        msg.info.origin.orientation.w = 1.0;
        msg.data.assign (height_data,
                         height_data + (static_cast<size_t> (width) * height));

        {
            std::lock_guard<std::mutex> lock (reg.mutex);
            auto it = reg.planners.find (handle);
            if (it == reg.planners.end ()) {
                writeError (err_buf, err_buf_len, "unknown planner handle");
                return kStatusArgError;
            }
            host = it->second.host;
            it->second.cached_height_grid = msg;
            it->second.has_height_grid    = true;
        }

        msg.header.stamp = host->node ()->now ();
        auto pub = host->node ()->create_publisher<nav_msgs::msg::OccupancyGrid> (
            "/height_grid",
            rclcpp::QoS (rclcpp::KeepLast (1)).reliable ().transient_local ());
        pub->publish (msg);
        rclcpp::spin_some (host->node ());
        return 0;
    } catch (const std::exception &ex) {
        writeError (err_buf, err_buf_len, ex.what ());
        return kStatusInternalError;
    }
}

}  // extern "C"
