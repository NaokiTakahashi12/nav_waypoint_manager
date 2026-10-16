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
#include <unordered_map>
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
#include <nav_waypoint_msgs/srv/reserve_route.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <nav_waypoint_server/nav_route_server_node_params.hpp>

#include "dataset_io.hpp"
#include "async_saver.hpp"

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
    reserve_client_ = create_client<nav_waypoint_msgs::srv::ReserveRoute>(
      "nav_waypoint_server/reserve_route");
    waypoint_subscription_ = create_subscription<nav_waypoint_msgs::msg::Waypoints>(
      "nav_waypoint_server/waypoints", rclcpp::QoS(1).transient_local(),
      [this](nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr ws) {
        try {
          nav_waypoint_msgs::validateWaypoints(*ws, params_.waypoint_frame_id);
          const auto active = route_index_ >= 0 &&
          route_index_ < static_cast<int>(route_.route.size()) ? route_.route[route_index_] : "";
          const auto old = waypoint_index_.find(active);
          for (const auto & w : ws->waypoints) {
            if (w.name == active && (old == waypoint_index_.end() ||
            !nav_waypoint_msgs::sameWaypoint(old->second, w)))
            {
              force_goal_ = true;
              if (old == waypoint_index_.end() || old->second.pose != w.pose ||
              reachSettings(old->second) != reachSettings(w))
              {
                waiting_resume_ = false;
                if (!startup_wait_) {resume_pending_ = false;}
              }
            }
          }
          waypoints_ = ws;
          waypoint_index_.clear();
          reach_settings_.clear();
          for (const auto & w : ws->waypoints) {
            waypoint_index_.emplace(w.name, w);
            reach_settings_.emplace(w.name, reachSettings(w));
          }
          refreshRouteValidation();
          if (!references_ready_) {sync_requested_ = true;}
        } catch (const std::exception & e) {
          // Do not continue using an old dataset after receiving an invalid replacement.
          waypoints_.reset();
          waypoint_index_.clear();
          route_error_ = e.what();
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
      "~/set_route", [this](rclcpp::Service<nav_waypoint_msgs::srv::SetRoute>::SharedPtr service,
      std::shared_ptr<rmw_request_id_t> header,
      nav_waypoint_msgs::srv::SetRoute::Request::SharedPtr request) {
        auto reply = [service, header](bool success, const std::string & message) {
          nav_waypoint_msgs::srv::SetRoute::Response response;
          response.success = success;
          response.message = message;
          service->send_response(*header, response);
        };
        try {
          if (reservation_pending_ || !reserve_client_->service_is_ready()) {
            throw std::invalid_argument("Route coordination unavailable or busy; retry later");
          }
          if (!request->route.header.frame_id.empty() &&
          request->route.header.frame_id != params_.waypoint_frame_id)
          {
            throw std::invalid_argument("Route frame mismatch");
          }
          validateRoute(request->route);
          reserveRoute(request->route, true, reply);
        } catch (const std::exception & e) {reply(false, e.what());}
      });
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save", [this](rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
      std::shared_ptr<rmw_request_id_t> header, std_srvs::srv::Trigger::Request::SharedPtr) {
        const auto snapshot = route_;
        const auto frame = params_.waypoint_frame_id;
        const auto path = params_.route_save_file.empty() ?
        params_.route_file : params_.route_save_file;
        auto reply = [service, header](bool success, const std::string & message) {
          std_srvs::srv::Trigger::Response response;
          response.success = success;
          response.message = message;
          try {service->send_response(*header, response);} catch (const std::exception &) {}
        };
        try {validateRoute(snapshot);} catch (const std::exception & e) {
          reply(false, e.what());
          return;
        }
        if (!saver_.submit(
          [snapshot, frame, path, reply]() {
            try {
              auto root = outputDataset(path, frame);
              YAML::Node entries(YAML::NodeType::Sequence);
              for (const auto & name : snapshot.route) {entries.push_back(name); }
              root["waypoint_manager"]["route"] = entries;
              atomicSave(path, root);
              reply(true, "Saved route");
            } catch (const std::exception & e) {reply(false, e.what());}
          })) {reply(false, "Save queue is full; retry later");}
      });
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / params_.reach_observe_frequency));
    timer_ = create_wall_timer(duration, [this]() {observe();});
    publishRoute();
    setProgress(route_index_ < 0 ? "disabled" : "startup_wait", "");
  }

private:
  int route_index_{0};
  bool force_goal_{true};
  std::chrono::steady_clock::time_point next_goal_{};
  bool references_ready_{false};
  bool reservation_pending_{false};
  bool sync_requested_{true};
  std::string route_error_;
  std::unordered_map<std::string, nav_waypoint_msgs::msg::Waypoint> waypoint_index_;
  std::unordered_map<std::string, std::pair<double, bool>> reach_settings_;
  rclcpp::Client<nav_waypoint_msgs::srv::ReserveRoute>::SharedPtr reserve_client_;
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
  AsyncSaver saver_;

  void validateRoute(const nav_waypoint_msgs::msg::Route & route) const
  {
    if (!waypoints_ && !route.route.empty()) {
      throw std::invalid_argument("Waypoints have not been received");
    }
    for (const auto & name : route.route) {
      if (name.empty() || !waypoint_index_.count(name)) {
        throw std::invalid_argument("Unknown route waypoint: " + name);
      }
    }
  }

  std::pair<double, bool> reachSettings(const nav_waypoint_msgs::msg::Waypoint & w) const
  {
    double radius = params_.reach_position_threshold;
    bool stop = false;
    for (const auto & property : w.properties) {
      if (property.key == "goal_reached_radius") {
        radius = nav_waypoint_msgs::positiveNumber(property.value);
      } else if (property.key == "stop_with_resume") {
        stop = property.value == "true";
      }
    }
    return {radius, stop};
  }

  void refreshRouteValidation()
  {
    route_error_.clear();
    try {validateRoute(route_);} catch (const std::exception & e) {route_error_ = e.what();}
  }

  void reserveRoute(
    const nav_waypoint_msgs::msg::Route & route, bool edit,
    std::function<void(bool, const std::string &)> reply)
  {
    auto request = std::make_shared<nav_waypoint_msgs::srv::ReserveRoute::Request>();
    request->owner = get_fully_qualified_name();
    request->route = route;
    reservation_pending_ = true;
    sync_requested_ = false;
    reserve_client_->async_send_request(
      request,
      [this, route, edit,
      reply](rclcpp::Client<nav_waypoint_msgs::srv::ReserveRoute>::SharedFuture f) {
        reservation_pending_ = false;
        const auto result = f.get();
        if (result->success) {
          references_ready_ = true;
          if (edit && route.route != route_.route) {
            route_ = route;
            route_.header.frame_id = params_.waypoint_frame_id;
            publishRoute();
            reset();
          }
          refreshRouteValidation();
        }
        if (reply) {reply(result->success, result->message);}
      });
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
    force_goal_ = true;
    startup_wait_ = true;
    waiting_resume_ = false;
    resume_pending_ = false;
    setProgress("startup_wait", "");
  }

  void publishGoal(const nav_waypoint_msgs::msg::Waypoint & waypoint)
  {
    const auto current = std::chrono::steady_clock::now();
    if (!force_goal_ && current < next_goal_) {return;}
    auto goal = waypoint;
    goal.frame_id = params_.waypoint_frame_id;
    goal.stamp = now();
    goal_publisher_->publish(goal);
    force_goal_ = false;
    next_goal_ = current + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        std::max<int64_t>(1, params_.publish_goal_with_reach_observe) /
        params_.reach_observe_frequency));
  }

  void observe()
  {
    if (sync_requested_ && !reservation_pending_ && reserve_client_->service_is_ready()) {
      reserveRoute(route_, false, {});
    }
    if (reservation_pending_ || !references_ready_) {
      setProgress("blocked", "Waiting for route reference coordination");
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
    if (!route_error_.empty()) {
      setProgress("blocked", route_error_);
      return;
    }
    if (startup_wait_ && !resume_pending_) {
      setProgress("startup_wait", "Send release_resume=true to start");
      return;
    }
    const auto & name = route_.route[route_index_];
    const auto found = waypoint_index_.find(name);
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        params_.waypoint_frame_id, params_.robot_frame_id, tf2::TimePointZero);
      if (tf.header.stamp.sec != 0 || tf.header.stamp.nanosec != 0) {
        const double age =
          (now() - rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type())).seconds();
        if (age > params_.max_transform_age || age < -params_.future_transform_tolerance) {
          if (!startup_wait_) {publishGoal(found->second);}
          setProgress("blocked", "Robot transform timestamp is stale or in the future");
          return;
        }
      }
    } catch (const tf2::TransformException & e) {
      if (!startup_wait_) {
        publishGoal(found->second);
      }
      setProgress("blocked", e.what());
      return;  // Retain a pending resume until TF is available again.
    }
    const auto settings = reach_settings_.at(name);
    const double radius = settings.first;
    const bool stop = settings.second;
    const double dx = found->second.pose.position.x - tf.transform.translation.x;
    const double dy = found->second.pose.position.y - tf.transform.translation.y;
    const double dz = params_.evalute_only_xy_reach ? 0.0 :
      found->second.pose.position.z - tf.transform.translation.z;
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
      force_goal_ = true;
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
    publishGoal(found->second);
    setProgress(waiting_resume_ ? "waiting_resume" : "running", "");
  }
};
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavRouteServer)
