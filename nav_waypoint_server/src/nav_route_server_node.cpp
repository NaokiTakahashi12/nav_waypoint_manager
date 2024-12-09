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

#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>
#include <nav_waypoint_msgs/msg/waypoints.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <std_srvs/srv/empty.hpp>

#include <nav_route_server_node_params.hpp>

namespace nav_waypoint_server
{
class NavRouteServer : public rclcpp::Node
{
public:
  explicit NavRouteServer(const rclcpp::NodeOptions &);
  ~NavRouteServer();

private:
  const int route_reset_index_;
  int route_index_;
  int publish_goal_index_;
  bool release_resume_state_;

  std::mutex waypoints_mutex_;
  std::mutex route_index_mutex_;

  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoint>::SharedPtr waypoint_publisher_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Route>::SharedPtr route_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoints_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr release_resume_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_route_service_;
  rclcpp::TimerBase::SharedPtr reach_waypoint_observer_timer_;

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_waypoint_msgs::msg::Route route_;
  nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr waypoints_;

  std::unique_ptr<nav_route_server::ParamListener> param_listener_;
  nav_route_server::Params params_;

  void waypointsSubscribeCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr &);
  void resetRouteServiceCallback(
    const std_srvs::srv::Empty::Request::ConstSharedPtr &,
    const std_srvs::srv::Empty::Response::SharedPtr &);
  void releaseResumeCallback(const std_msgs::msg::Bool::ConstSharedPtr &);
  void reachWaypointObserverCallback();

  void publishRoute();
  void publishLatestWaypoint(const int waypoint_index);
  nav_waypoint_msgs::msg::Route loadRoute(const std::string & file_path);
  bool isLoadableRoute(const std::string & file_path);
};

NavRouteServer::NavRouteServer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_route_server",
    rclcpp::NodeOptions(node_options).use_intra_process_comms(false)),
  route_reset_index_(-1),
  route_index_(route_reset_index_),
  publish_goal_index_(0),
  release_resume_state_(false),
  waypoint_publisher_(nullptr),
  route_publisher_(nullptr),
  waypoints_subscriber_(nullptr),
  release_resume_subscriber_(nullptr),
  reset_route_service_(nullptr),
  reach_waypoint_observer_timer_(nullptr),
  tf_listener_(nullptr),
  tf_buffer_(nullptr),
  waypoints_(nullptr),
  param_listener_(nullptr),
  params_()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());

  param_listener_ = std::make_unique<nav_route_server::ParamListener>(
    this->get_node_parameters_interface()
  );
  params_ = param_listener_->get_params();

  if (isLoadableRoute(params_.route_file)) {
    route_ = loadRoute(params_.route_file);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Can't open route file: " << params_.route_file);
  }
  route_.header.stamp = this->get_clock()->now();
  route_.header.frame_id = params_.waypoint_frame_id;

  if (params_.route_index_auto_reset) {
    route_index_ = 0;
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  waypoint_publisher_ = this->create_publisher<nav_waypoint_msgs::msg::Waypoint>(
    "~/waypoint",
    rclcpp::QoS(2)
  );
  route_publisher_ = this->create_publisher<nav_waypoint_msgs::msg::Route>(
    "~/route",
    rclcpp::QoS(5).transient_local()
  );
  waypoints_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoints>(
    "nav_waypoint_server/waypoints",
    rclcpp::QoS(5).transient_local(),
    std::bind(
      &NavRouteServer::waypointsSubscribeCallback,
      this,
      std::placeholders::_1
    )
  );
  reset_route_service_ = this->create_service<std_srvs::srv::Empty>(
    "~/reset",
    std::bind(
      &NavRouteServer::resetRouteServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  release_resume_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/release_resume",
    rclcpp::QoS(3),
    std::bind(
      &NavRouteServer::releaseResumeCallback,
      this,
      std::placeholders::_1
    )
  );
  const unsigned int reach_waypoint_observer_duration_milliseconds =
    1e3 / params_.reach_observe_frequency;
  reach_waypoint_observer_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(reach_waypoint_observer_duration_milliseconds),
    std::bind(
      &NavRouteServer::reachWaypointObserverCallback,
      this
    )
  );
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize successfully " << this->get_name());
  publishRoute();
}

NavRouteServer::~NavRouteServer()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void NavRouteServer::waypointsSubscribeCallback(
  const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> waypoint_lock{waypoints_mutex_};
  RCLCPP_INFO(this->get_logger(), "Recived new waypoint");
  waypoints_ = msg;
  route_index_ = 0;
}

void NavRouteServer::resetRouteServiceCallback(
  const std_srvs::srv::Empty::Request::ConstSharedPtr &,
  const std_srvs::srv::Empty::Response::SharedPtr &)
{
  std::lock_guard<std::mutex> route_index_lock{route_index_mutex_};
  route_index_ = 0;
  RCLCPP_INFO(this->get_logger(), "Resetted route index");
}

void NavRouteServer::releaseResumeCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  release_resume_state_ = msg->data;
}

void NavRouteServer::reachWaypointObserverCallback()
{
  static bool startup_resume = true;

  const bool release_resume_state = release_resume_state_;
  int waypoint_index = route_reset_index_;
  release_resume_state_ = false;

  if (release_resume_state) {
    startup_resume = false;
  }
  if (route_.route.size() < 1) {
    RCLCPP_WARN(this->get_logger(), "Route size is ZERO");
    return;
  }
  if (startup_resume) {
    RCLCPP_INFO(this->get_logger(), "Startup resume now; please interact");
    return;
  }
  //! waypoint_lock scope
  {
    std::lock_guard<std::mutex> waypoint_lock{waypoints_mutex_};
    if (!waypoints_) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Not recived waypoints");
      return;
    }
    if (route_index_ < 0 || route_index_ >= static_cast<int>(route_.route.size())) {
      RCLCPP_WARN(this->get_logger(), "Not ready route index");
      return;
    }
    const std::string waypoint_name = route_.route[route_index_];
    int waypoint_counter = 0;

    for (const auto & w : waypoints_->waypoints) {
      if (w.name == waypoint_name) {
        waypoint_index = waypoint_counter;
        break;
      }
      waypoint_counter++;
    }
    if (waypoint_index < 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Not exits waypoint: " << waypoint_name);
      RCLCPP_WARN_STREAM(this->get_logger(), "Skip waypoint: " << waypoint_name);
      route_index_++;
      publish_goal_index_ = 0;
      return;
    }
    if (waypoint_index >= static_cast<int>(waypoints_->waypoints.size())) {
      RCLCPP_INFO(this->get_logger(), "Finish route");
      waypoint_index = route_reset_index_;
      return;
    }
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        params_.waypoint_frame_id,
        params_.robot_frame_id,
        tf2::TimePointZero
      );
    } catch (const tf2::TransformException & tfe) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Could not transform "
          << params_.waypoint_frame_id
          << " to "
          << params_.robot_frame_id);
      return;
    }
    Eigen::Vector3d robot_position;
    Eigen::Vector3d waypoint_position;
    Eigen::Vector3d dist_position;
    const geometry_msgs::msg::Pose waypoint_pose = waypoints_->waypoints[waypoint_index].pose;

    if (params_.evalute_only_xy_reach) {
      robot_position.x() = tf_stamped.transform.translation.x;
      robot_position.y() = tf_stamped.transform.translation.y;
      robot_position.z() = 0.0;
      waypoint_position.x() = waypoint_pose.position.x;
      waypoint_position.y() = waypoint_pose.position.y;
      waypoint_position.z() = 0.0;
    } else {
      robot_position.x() = tf_stamped.transform.translation.x;
      robot_position.y() = tf_stamped.transform.translation.y;
      robot_position.z() = tf_stamped.transform.translation.z;
      waypoint_position.x() = waypoint_pose.position.x;
      waypoint_position.y() = waypoint_pose.position.y;
      waypoint_position.z() = waypoint_pose.position.z;
    }

    dist_position = waypoint_position - robot_position;
    const double goal_dist_norm = dist_position.lpNorm<2>();
    double goal_reached_radius = params_.reach_position_threshold;
    for (const auto & property : waypoints_->waypoints[waypoint_index].properties) {
      if (property.key == "goal_reached_radius") {
        goal_reached_radius = std::stof(property.value);
        break;
      }
    }
    if (goal_reached_radius > goal_dist_norm) {
      bool resume_enabled = false;
      for (const auto & property : waypoints_->waypoints[waypoint_index].properties) {
        if (property.key == "stop_with_resume") {
          if (property.value == "true") {
            resume_enabled = true;
          }
          break;
        }
      }
      if (resume_enabled && !release_resume_state) {
        RCLCPP_INFO(this->get_logger(), "Resume state now; please interact");
        return;
      }
      route_index_++;
      publish_goal_index_ = 0;
      return;
    }
  }
  if (publish_goal_index_ == 0) {
    publishLatestWaypoint(waypoint_index);
    RCLCPP_INFO_STREAM(this->get_logger(), "Next waypoint is " << waypoints_->waypoints[waypoint_index].name);
  }
  if (publish_goal_index_ >= params_.publish_goal_with_reach_observe) {
    publish_goal_index_ = 0;
  } else {
    publish_goal_index_++;
  }
}

void NavRouteServer::publishRoute()
{
  nav_waypoint_msgs::msg::Route::UniquePtr msg;
  msg = std::make_unique<nav_waypoint_msgs::msg::Route>(route_);
  route_publisher_->publish(std::move(msg));
}

void NavRouteServer::publishLatestWaypoint(const int waypoint_index)
{
  if (waypoint_index < 0) {
    RCLCPP_WARN(this->get_logger(), "Incorrect index");
    return;
  }
  std::lock_guard<std::mutex> route_index_lock{route_index_mutex_};

  nav_waypoint_msgs::msg::Waypoint::UniquePtr msg;
  msg = std::make_unique<nav_waypoint_msgs::msg::Waypoint>(
    waypoints_->waypoints[waypoint_index]
  );
  msg->stamp = this->get_clock()->now();
  waypoint_publisher_->publish(std::move(msg));
}

nav_waypoint_msgs::msg::Route NavRouteServer::loadRoute(const std::string & file_path)
{
  nav_waypoint_msgs::msg::Route ret_route;
  YAML::Node yaml_node = YAML::LoadFile(file_path);

  for (const auto & r : yaml_node["waypoint_manager"]["route"]) {
    ret_route.route.push_back(r.as<std::string>());
  }
  return ret_route;
}

bool NavRouteServer::isLoadableRoute(const std::string & file_path)
{
  std::ifstream f;
  f.open(file_path);
  const bool is_open = f.is_open();
  f.close();
  return is_open;
}
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavRouteServer)
