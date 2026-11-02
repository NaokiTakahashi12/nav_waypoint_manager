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
#include <vector>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <nav_waypoint_msgs/msg/route_progress.hpp>
#include <nav_waypoint_msgs/msg/waypoint_route_configuration.hpp>
#include <nav_waypoint_msgs/msg/waypoint_goal.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <nav_waypoint_server/nav_route_server_node_params.hpp>


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
      params_.publish_goal_with_reach_observe < 0 ||
      !std::isfinite(params_.max_transform_age) || params_.max_transform_age <= 0.0 ||
      !std::isfinite(params_.future_transform_tolerance) ||
      params_.future_transform_tolerance < 0.0 ||
      std::max<int64_t>(1, params_.publish_goal_with_reach_observe) /
      params_.reach_observe_frequency >=
      std::chrono::duration<double>(std::chrono::nanoseconds::max()).count())
    {
      throw std::invalid_argument(
              "Invalid frame, observation frequency, radius or publish interval");
    }
    route_.header.frame_id = params_.waypoint_frame_id;
    route_index_ = params_.route_index_auto_reset ? 0 : -1;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    goal_publisher_ = create_publisher<nav_waypoint_msgs::msg::WaypointGoal>("~/waypoint", 2);
    route_publisher_ = create_publisher<nav_waypoint_msgs::msg::Route>(
      "~/route", rclcpp::QoS(1).transient_local());
    progress_publisher_ = create_publisher<nav_waypoint_msgs::msg::RouteProgress>(
      "~/progress", rclcpp::QoS(1).transient_local());
    configuration_subscription_ =
      create_subscription<nav_waypoint_msgs::msg::WaypointRouteConfiguration>(
      "nav_waypoint_server/configuration", rclcpp::QoS(1).transient_local(),
      [this](nav_waypoint_msgs::msg::WaypointRouteConfiguration::ConstSharedPtr configuration) {
        applyConfiguration(*configuration);
      });
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset", [this](std_srvs::srv::Empty::Request::ConstSharedPtr,
      std_srvs::srv::Empty::Response::SharedPtr) {reset();});
    resume_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "~/release_resume", 3, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        // Commands received while moving do not release a future waypoint.
        if (msg->data && (phase_ == Phase::Start || phase_ == Phase::Waiting)) {
          resume_pending_ = true;
        }
      });
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.reach_observe_frequency));
    timer_ = create_wall_timer(duration, [this]() {observe();});
    publishConfiguredRoute();
    setProgress(route_index_ < 0 ? "disabled" : "blocked", "Waiting for configuration");
  }

private:
  int route_index_{0};
  bool force_goal_{true};
  std::chrono::steady_clock::time_point next_goal_{};
  bool configuration_received_{false};
  uint64_t goal_sequence_{0};
  const std::string instance_id_{std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count())};
  std::string route_error_;
  // Execution state contains only locations selected by the configured route.
  std::vector<nav_waypoint_msgs::msg::Waypoint> route_waypoints_;
  std::vector<std::pair<double, bool>> route_reach_settings_;
  enum class Phase {Start, Moving, Waiting};
  Phase phase_{Phase::Start};
  bool resume_pending_{false};
  nav_waypoint_msgs::msg::Route route_;
  nav_waypoint_msgs::msg::RouteProgress progress_;
  std::unique_ptr<nav_route_server::ParamListener> param_listener_;
  nav_route_server::Params params_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::WaypointGoal>::SharedPtr goal_publisher_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Route>::SharedPtr route_publisher_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::RouteProgress>::SharedPtr progress_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::WaypointRouteConfiguration>::SharedPtr
    configuration_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  void applyConfiguration(const nav_waypoint_msgs::msg::WaypointRouteConfiguration & configuration)
  {
    try {
      auto next_route_waypoints = nav_waypoint_msgs::resolveRouteWaypoints(
        configuration, params_.waypoint_frame_id);
      std::vector<std::pair<double, bool>> next_reach_settings;
      for (const auto & waypoint : next_route_waypoints) {
        next_reach_settings.push_back(
          nav_waypoint_msgs::reachSettings(waypoint, params_.reach_position_threshold));
      }
      const bool route_changed = !configuration_received_ ||
        configuration.route.route != route_.route;
      if (!route_changed && route_index_ >= 0 &&
        route_index_ < static_cast<int>(next_route_waypoints.size()))
      {
        const auto index = static_cast<std::size_t>(route_index_);
        if (index >= route_waypoints_.size() ||
          !nav_waypoint_msgs::sameWaypoint(route_waypoints_[index], next_route_waypoints[index]))
        {
          newGoal();
          if (phase_ != Phase::Start && (index >= route_waypoints_.size() ||
            route_waypoints_[index].pose != next_route_waypoints[index].pose ||
            route_reach_settings_[index] != next_reach_settings[index]))
          {
            phase_ = Phase::Moving;
            resume_pending_ = false;
          }
        }
      }
      route_ = configuration.route;
      route_waypoints_ = std::move(next_route_waypoints);
      route_reach_settings_ = std::move(next_reach_settings);
      configuration_received_ = true;
      route_error_.clear();
      if (route_changed) {publishConfiguredRoute();}
      if (route_changed && route_index_ >= 0) {reset();}
    } catch (const std::exception & e) {
      // Never execute a previous snapshot after an invalid replacement arrives.
      route_waypoints_.clear();
      route_reach_settings_.clear();
      route_error_ = e.what();
      setProgress("blocked", route_error_);
    }
  }

  void publishConfiguredRoute()
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
    newGoal();
    phase_ = Phase::Start;
    resume_pending_ = false;
    setProgress("startup_wait", "");
  }

  void newGoal()
  {
    force_goal_ = true;
    ++goal_sequence_;
  }

  void publishGoal(const nav_waypoint_msgs::msg::Waypoint & waypoint)
  {
    const auto current = std::chrono::steady_clock::now();
    if (!force_goal_ && current < next_goal_) {return;}
    auto goal = waypoint;
    goal.frame_id = params_.waypoint_frame_id;
    goal.stamp = now();
    nav_waypoint_msgs::msg::WaypointGoal command;
    command.goal_id = instance_id_ + "/" + std::to_string(goal_sequence_);
    command.waypoint = goal;
    goal_publisher_->publish(command);
    force_goal_ = false;
    next_goal_ = current + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        std::max<int64_t>(1, params_.publish_goal_with_reach_observe) /
        params_.reach_observe_frequency));
  }

  void observe()
  {
    if (!route_error_.empty()) {
      setProgress("blocked", route_error_);
      return;
    }
    if (!configuration_received_) {
      setProgress("blocked", "Waiting for configuration");
      return;
    }
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
    if (phase_ == Phase::Start && !resume_pending_) {
      setProgress("startup_wait", "Send release_resume=true to start");
      return;
    }
    const auto & target_waypoint = route_waypoints_[route_index_];
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        params_.waypoint_frame_id, params_.robot_frame_id, tf2::TimePointZero);
      if (tf.header.stamp.sec != 0 || tf.header.stamp.nanosec != 0) {
        const double age =
          (now() - rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type())).seconds();
        if (age > params_.max_transform_age || age < -params_.future_transform_tolerance) {
          if (phase_ != Phase::Start) {publishGoal(target_waypoint);}
          setProgress("blocked", "Robot transform timestamp is stale or in the future");
          return;
        }
      }
    } catch (const tf2::TransformException & e) {
      if (phase_ != Phase::Start) {
        publishGoal(target_waypoint);
      }
      setProgress("blocked", e.what());
      return;  // Retain a pending resume until TF is available again.
    }
    const auto settings = route_reach_settings_[route_index_];
    const double radius = settings.first;
    const bool stop = settings.second;
    const double dx = target_waypoint.pose.position.x - tf.transform.translation.x;
    const double dy = target_waypoint.pose.position.y - tf.transform.translation.y;
    const double dz = params_.evalute_only_xy_reach ? 0.0 :
      target_waypoint.pose.position.z - tf.transform.translation.z;
    const double distance = std::hypot(std::hypot(dx, dy), dz);
    if (!std::isfinite(distance)) {
      setProgress("blocked", "Non-finite robot transform");
      return;
    }
    if (phase_ == Phase::Start) {
      phase_ = Phase::Moving;
      resume_pending_ = false;  // Starting never releases the first stop waypoint.
    }
    const bool reached = distance <= radius;
    if (reached && (!stop || (phase_ == Phase::Waiting && resume_pending_))) {
      route_index_++;
      newGoal();
      phase_ = Phase::Moving;
      resume_pending_ = false;
      setProgress(
        route_index_ == static_cast<int>(route_.route.size()) ? "completed" : "running", "");
      return;
    }
    if (reached && stop) {
      phase_ = Phase::Waiting;
    }
    // Continue issuing the same position even while waiting or after drifting
    // outside the radius. The receiver remains responsible for robot motion.
    publishGoal(target_waypoint);
    setProgress(phase_ == Phase::Waiting ? "waiting_resume" : "running", "");
  }
};
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavRouteServer)
