#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
  NavRouteServer(const rclcpp::NodeOptions &);
  ~NavRouteServer();

private:
  const int m_route_reset_index;
  int m_route_index;
  int m_publish_goal_index;

  std::mutex m_waypoints_mutex;
  std::mutex m_route_index_mutex;

  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoint>::SharedPtr m_waypoint_publisher;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Route>::SharedPtr m_route_publisher;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr m_waypoints_subscriber;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_reset_route_service;
  rclcpp::TimerBase::SharedPtr m_reach_waypoint_observer_timer;

  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

  nav_waypoint_msgs::msg::Route m_route;
  nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr m_waypoints;

  std::unique_ptr<nav_route_server::ParamListener> m_param_listener;
  nav_route_server::Params m_params;

  void waypointsSubscribeCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr &);
  void resetRouteServiceCallback(
    const std_srvs::srv::Empty::Request::ConstSharedPtr &,
    const std_srvs::srv::Empty::Response::SharedPtr &);
  void reachWaypointObserverCallback();

  void publishRoute();
  void publishLatestWaypoint(const int waypoint_index);
  nav_waypoint_msgs::msg::Route loadRoute(const std::string & file_path);
  bool isLoadableRoute(const std::string & file_path);
};

NavRouteServer::NavRouteServer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_route_server", node_options),
  m_route_reset_index(-1),
  m_route_index(m_route_reset_index),
  m_publish_goal_index(0),
  m_waypoint_publisher(nullptr),
  m_route_publisher(nullptr),
  m_waypoints_subscriber(nullptr),
  m_reset_route_service(nullptr),
  m_reach_waypoint_observer_timer(nullptr),
  m_tf_listener(nullptr),
  m_tf_buffer(nullptr),
  m_waypoints(nullptr),
  m_param_listener(nullptr),
  m_params()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());

  m_param_listener = std::make_unique<nav_route_server::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = m_param_listener->get_params();

  if (isLoadableRoute(m_params.route_file)) {
    m_route = loadRoute(m_params.route_file);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Can't open route file: " << m_params.route_file);
  }
  m_route.header.stamp = this->get_clock()->now();
  m_route.header.frame_id = m_params.waypoint_frame_id;

  if (m_params.route_index_auto_reset) {
    m_route_index = 0;
  }

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

  m_waypoint_publisher = this->create_publisher<nav_waypoint_msgs::msg::Waypoint>(
    "~/waypoint",
    rclcpp::QoS(2)
  );
  m_route_publisher = this->create_publisher<nav_waypoint_msgs::msg::Route>(
    "~/route",
    rclcpp::QoS(5).transient_local()
  );
  m_waypoints_subscriber = this->create_subscription<nav_waypoint_msgs::msg::Waypoints>(
    "nav_waypoint_server/waypoints",
    rclcpp::QoS(5).transient_local(),
    std::bind(
      &NavRouteServer::waypointsSubscribeCallback,
      this,
      std::placeholders::_1
    )
  );
  m_reset_route_service = this->create_service<std_srvs::srv::Empty>(
    "~/reset",
    std::bind(
      &NavRouteServer::resetRouteServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  const unsigned int reach_waypoint_observer_duration_milliseconds =
    1e3 / m_params.reach_observe_frequency;
  m_reach_waypoint_observer_timer = this->create_wall_timer(
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
  std::lock_guard<std::mutex> waypoint_lock{m_waypoints_mutex};
  RCLCPP_INFO(this->get_logger(), "Recived new waypoint");
  m_waypoints = msg;
}

void NavRouteServer::resetRouteServiceCallback(
  const std_srvs::srv::Empty::Request::ConstSharedPtr &,
  const std_srvs::srv::Empty::Response::SharedPtr &)
{
  std::lock_guard<std::mutex> route_index_lock{m_route_index_mutex};
  m_route_index = 0;
  RCLCPP_INFO(this->get_logger(), "Resetted route index");
}

void NavRouteServer::reachWaypointObserverCallback()
{
  int waypoint_index = m_route_reset_index;
  double goal_dist_norm;

  if (m_route.route.size() < 1) {
    RCLCPP_WARN(this->get_logger(), "Route size is ZERO");
    return;
  }
  //! waypoint_lock scope
  {
    std::lock_guard<std::mutex> waypoint_lock{m_waypoints_mutex};
    if (not m_waypoints) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Not recived waypoints");
      return;
    }
    if (m_route_index < 0 || m_route_index >= static_cast<int>(m_route.route.size())) {
      RCLCPP_WARN(this->get_logger(), "Not ready route index");
      return;
    }
    const std::string waypoint_name = m_route.route[m_route_index];
    int waypoint_counter = 0;

    for (const auto & w : m_waypoints->waypoints) {
      if (w.name == waypoint_name) {
        waypoint_index = waypoint_counter;
        break;
      }
      waypoint_counter++;
    }
    if (waypoint_index < 0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Not exits waypoint: " << waypoint_name);
      RCLCPP_WARN_STREAM(this->get_logger(), "Skip waypoint: " << waypoint_name);
      m_route_index++;
      m_publish_goal_index = 0;
      return;
    }
    if (waypoint_index >= static_cast<int>(m_waypoints->waypoints.size())) {
      RCLCPP_INFO(this->get_logger(), "Finish route");
      waypoint_index = m_route_reset_index;
      return;
    }
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = m_tf_buffer->lookupTransform(
        m_params.waypoint_frame_id,
        m_params.robot_frame_id,
        tf2::TimePointZero
      );
    } catch (const tf2::TransformException & tfe) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Could not transform "
          << m_params.waypoint_frame_id
          << " to "
          << m_params.robot_frame_id);
      return;
    }
    Eigen::Vector3d robot_position;
    Eigen::Vector3d waypoint_position;
    Eigen::Vector3d dist_position;
    const geometry_msgs::msg::Pose waypoint_pose = m_waypoints->waypoints[waypoint_index].pose;

    robot_position.x() = tf_stamped.transform.translation.x;
    robot_position.y() = tf_stamped.transform.translation.y;
    robot_position.z() = tf_stamped.transform.translation.z;
    waypoint_position.x() = waypoint_pose.position.x;
    waypoint_position.y() = waypoint_pose.position.y;
    waypoint_position.z() = waypoint_pose.position.z;

    dist_position = waypoint_position - robot_position;
    goal_dist_norm = dist_position.lpNorm<2>();
  }
  if (m_params.reach_position_threshold > goal_dist_norm) {
    m_route_index++;
    m_publish_goal_index = 0;
    return;
  }
  if (m_publish_goal_index == 0) {
    publishLatestWaypoint(waypoint_index);
  }
  if (m_publish_goal_index >= m_params.publish_goal_with_reach_observe) {
    m_publish_goal_index = 0;
  } else {
    m_publish_goal_index++;
  }
}

void NavRouteServer::publishRoute()
{
  nav_waypoint_msgs::msg::Route::UniquePtr msg;
  msg = std::make_unique<nav_waypoint_msgs::msg::Route>(m_route);
  m_route_publisher->publish(std::move(msg));
}

void NavRouteServer::publishLatestWaypoint(const int waypoint_index)
{
  if (waypoint_index < 0) {
    RCLCPP_WARN(this->get_logger(), "Incorrect index");
    return;
  }
  std::lock_guard<std::mutex> route_index_lock{m_route_index_mutex};

  nav_waypoint_msgs::msg::Waypoint::UniquePtr msg;
  msg = std::make_unique<nav_waypoint_msgs::msg::Waypoint>(
    m_waypoints->waypoints[waypoint_index]
  );
  msg->stamp = this->get_clock()->now();
  m_waypoint_publisher->publish(std::move(msg));
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
