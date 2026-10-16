// Copyright (c) 2023 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <nav_waypoint_msgs/msg/route_progress.hpp>
#include <nav_waypoint_msgs/srv/set_route.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <nav_waypoint_server/nav_route_server_node_params.hpp>

#include "dataset_io.hpp"

namespace nav_waypoint_server
{
class NavRouteServer : public rclcpp::Node
{
public:
  explicit NavRouteServer(const rclcpp::NodeOptions & options)
  : Node("nav_route_server", rclcpp::NodeOptions(options).use_intra_process_comms(false))
  {
    param_listener_ = std::make_unique<nav_route_server::ParamListener>(
      get_node_parameters_interface());
    params_ = param_listener_->get_params();
    if (params_.waypoint_frame_id.empty() || params_.robot_frame_id.empty() ||
      !std::isfinite(params_.reach_observe_frequency) || params_.reach_observe_frequency <= 0.0 ||
      params_.reach_observe_frequency > 1000.0 ||
      1.0 / params_.reach_observe_frequency >=
      std::chrono::duration<double>(std::chrono::nanoseconds::max()).count() ||
      !std::isfinite(params_.reach_position_threshold) || params_.reach_position_threshold <= 0.0 ||
      params_.publish_goal_with_reach_observe < 0)
    {
      throw std::invalid_argument(
              "Invalid frame, observation frequency, radius or publish interval");
    }
    route_.header.frame_id = params_.waypoint_frame_id;
    if (fileExists(params_.route_file)) {
      const auto root = loadDataset(params_.route_file, params_.waypoint_frame_id);
      const auto entries = root["waypoint_manager"]["route"];
      if (!entries || !entries.IsSequence()) {
        throw std::invalid_argument("route must be a sequence");
      }
      for (const auto & entry : entries) {
        const auto name = entry.as<std::string>();
        if (name.empty()) {
          throw std::invalid_argument("Route waypoint names must not be empty");
        }
        route_.route.push_back(name);
      }
    } else {
      RCLCPP_WARN(get_logger(), "Starting with an empty route: %s", params_.route_file.c_str());
    }
    route_index_ = params_.route_index_auto_reset ? 0 : -1;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    goal_publisher_ = create_publisher<nav_waypoint_msgs::msg::Waypoint>("~/waypoint", 2);
    route_publisher_ = create_publisher<nav_waypoint_msgs::msg::Route>(
      "~/route", rclcpp::QoS(1).transient_local());
    progress_publisher_ = create_publisher<nav_waypoint_msgs::msg::RouteProgress>(
      "~/progress", rclcpp::QoS(1).transient_local());
    waypoint_subscription_ = create_subscription<nav_waypoint_msgs::msg::Waypoints>(
      "nav_waypoint_server/waypoints", rclcpp::QoS(1).transient_local(),
      [this](nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr ws) {
        try {
          nav_waypoint_msgs::validateWaypoints(*ws, params_.waypoint_frame_id);
          waypoints_ = ws;
          publish_goal_index_ = 0;  // Apply edits without changing route progress.
        } catch (const std::exception & e) {
          // Do not continue using an old dataset after receiving an invalid replacement.
          waypoints_.reset();
          setProgress("blocked", e.what());
        }
      });
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset", [this](std_srvs::srv::Empty::Request::ConstSharedPtr,
      std_srvs::srv::Empty::Response::SharedPtr) {reset();});
    resume_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "~/release_resume", 3, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        // Commands received while moving do not release a future waypoint.
        if (msg->data && (startup_wait_ || waiting_resume_)) {
          resume_pending_ = true;
        }
      });
    set_route_service_ = create_service<nav_waypoint_msgs::srv::SetRoute>(
      "~/set_route", [this](nav_waypoint_msgs::srv::SetRoute::Request::ConstSharedPtr request,
      nav_waypoint_msgs::srv::SetRoute::Response::SharedPtr response) {
        try {
          if (!request->route.header.frame_id.empty() &&
          request->route.header.frame_id != params_.waypoint_frame_id)
          {
            throw std::invalid_argument("Route frame mismatch");
          }
          validateRoute(request->route);
          route_ = request->route;
          route_.header.frame_id = params_.waypoint_frame_id;
          publishRoute();
          reset();
          response->success = true;
          response->message = "Route replaced; release_resume is required to start";
        } catch (const std::exception & e) {
          response->message = e.what();
        }
      });
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save", [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        try {
          validateRoute(route_);
          const auto path = params_.route_save_file.empty() ?
          params_.route_file : params_.route_save_file;
          auto root = outputDataset(path, params_.waypoint_frame_id);
          YAML::Node entries(YAML::NodeType::Sequence);
          for (const auto & name : route_.route) {
            entries.push_back(name);
          }
          root["waypoint_manager"]["route"] = entries;
          atomicSave(path, root);
          response->success = true;
          response->message = "Saved route";
        } catch (const std::exception & e) {
          response->message = e.what();
          RCLCPP_ERROR(get_logger(), "Save failed: %s", e.what());
        }
      });
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.reach_observe_frequency));
    timer_ = create_wall_timer(duration, [this]() {observe();});
    publishRoute();
    setProgress(route_index_ < 0 ? "disabled" : "startup_wait", "");
  }

private:
  int route_index_{0};
  int64_t publish_goal_index_{0};
  bool startup_wait_{true};
  bool waiting_resume_{false};
  bool resume_pending_{false};
  nav_waypoint_msgs::msg::Route route_;
  nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr waypoints_;
  nav_waypoint_msgs::msg::RouteProgress progress_;
  std::unique_ptr<nav_route_server::ParamListener> param_listener_;
  nav_route_server::Params params_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoint>::SharedPtr goal_publisher_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Route>::SharedPtr route_publisher_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::RouteProgress>::SharedPtr progress_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<nav_waypoint_msgs::srv::SetRoute>::SharedPtr set_route_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  void validateRoute(const nav_waypoint_msgs::msg::Route & route) const
  {
    if (!waypoints_ && !route.route.empty()) {
      throw std::invalid_argument("Waypoints have not been received");
    }
    for (const auto & name : route.route) {
      if (name.empty() || std::none_of(
          waypoints_->waypoints.begin(), waypoints_->waypoints.end(),
          [&name](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == name;}))
      {
        throw std::invalid_argument("Unknown route waypoint: " + name);
      }
    }
  }

  void publishRoute()
  {
    route_.header.stamp = now();
    route_publisher_->publish(route_);
  }

  void setProgress(const std::string & state, const std::string & reason)
  {
    const std::string name = route_index_ >= 0 &&
      route_index_ < static_cast<int>(route_.route.size()) ? route_.route[route_index_] : "";
    if (progress_.state == state && progress_.reason == reason &&
      progress_.route_index == route_index_ && progress_.route_size == route_.route.size() &&
      progress_.waypoint_name == name)
    {
      return;
    }
    progress_.header.frame_id = params_.waypoint_frame_id;
    progress_.header.stamp = now();
    progress_.state = state;
    progress_.reason = reason;
    progress_.route_index = route_index_;
    progress_.route_size = static_cast<uint32_t>(route_.route.size());
    progress_.waypoint_name = name;
    progress_publisher_->publish(progress_);
  }

  void reset()
  {
    route_index_ = 0;
    publish_goal_index_ = 0;
    startup_wait_ = true;
    waiting_resume_ = false;
    resume_pending_ = false;
    setProgress("startup_wait", "");
  }

  void publishGoal(const nav_waypoint_msgs::msg::Waypoint & waypoint)
  {
    if (publish_goal_index_ == 0) {
      auto goal = waypoint;
      goal.frame_id = params_.waypoint_frame_id;
      goal.stamp = now();
      goal_publisher_->publish(goal);
    }
    publish_goal_index_++;
    if (publish_goal_index_ >= std::max<int64_t>(1, params_.publish_goal_with_reach_observe)) {
      publish_goal_index_ = 0;
    }
  }

  void observe()
  {
    if (route_index_ < 0) {
      setProgress("disabled", "Call reset to enable the route");
      return;
    }
    if (route_.route.empty()) {
      setProgress("empty", "No route configured");
      return;
    }
    if (route_index_ >= static_cast<int>(route_.route.size())) {
      setProgress("completed", "All route positions passed; not navigation success");
      return;
    }
    try {
      validateRoute(route_);
    } catch (const std::exception & e) {
      setProgress("blocked", e.what());
      return;
    }
    if (startup_wait_ && !resume_pending_) {
      setProgress("startup_wait", "Send release_resume=true to start");
      return;
    }
    const auto & name = route_.route[route_index_];
    const auto found = std::find_if(
      waypoints_->waypoints.begin(), waypoints_->waypoints.end(),
      [&name](const nav_waypoint_msgs::msg::Waypoint & w) {return w.name == name;});
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        params_.waypoint_frame_id, params_.robot_frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      if (!startup_wait_) {
        publishGoal(*found);
      }
      setProgress("blocked", e.what());
      return;  // Retain a pending resume until TF is available again.
    }
    double radius = params_.reach_position_threshold;
    bool stop = false;
    for (const auto & property : found->properties) {
      if (property.key == "goal_reached_radius") {
        radius = nav_waypoint_msgs::positiveNumber(property.value);
      } else if (property.key == "stop_with_resume") {
        stop = property.value == "true";
      }
    }
    const double dx = found->pose.position.x - tf.transform.translation.x;
    const double dy = found->pose.position.y - tf.transform.translation.y;
    const double dz = params_.evalute_only_xy_reach ? 0.0 :
      found->pose.position.z - tf.transform.translation.z;
    const double distance = std::hypot(std::hypot(dx, dy), dz);
    if (!std::isfinite(distance)) {
      setProgress("blocked", "Non-finite robot transform");
      return;
    }
    if (startup_wait_) {
      startup_wait_ = false;
      resume_pending_ = false;  // Starting never releases the first stop waypoint.
    }
    const bool reached = distance <= radius;
    if (reached && (!stop || (waiting_resume_ && resume_pending_))) {
      route_index_++;
      publish_goal_index_ = 0;
      waiting_resume_ = false;
      resume_pending_ = false;
      setProgress(
        route_index_ == static_cast<int>(route_.route.size()) ? "completed" : "running", "");
      return;
    }
    if (reached && stop) {
      waiting_resume_ = true;
    }
    // Continue issuing the same position even while waiting or after drifting
    // outside the radius. The receiver remains responsible for robot motion.
    publishGoal(*found);
    setProgress(waiting_resume_ ? "waiting_resume" : "running", "");
  }
};
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavRouteServer)
