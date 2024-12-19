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

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <functional>
#include <vector>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>
#include <nav_waypoint_msgs/msg/waypoints.hpp>

#include <nav_waypoint_server_node_params.hpp>

namespace nav_waypoint_server
{
class NavWaypointServerNode : public rclcpp::Node
{
public:
  explicit NavWaypointServerNode(const rclcpp::NodeOptions &);
  ~NavWaypointServerNode();

private:
  nav_waypoint_msgs::msg::Waypoints waypoints_;

  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoints_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoint>::SharedPtr regist_waypoint_subscriber_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoint>::SharedPtr modify_waypoint_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_latest_waypoint_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_waypoints_service_;

  std::unique_ptr<ParamListener> param_listener_;
  nav_waypoint_server::Params params_;

  void registWaypointCallback(
    const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr &);
  void modifyWaypointCallback(
    const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr &);
  void saveWaypointsServiceCallback(
    const std_srvs::srv::Empty::Request::ConstSharedPtr &,
    const std_srvs::srv::Empty::Response::SharedPtr &);
  void publishWaypointsServiceCallback(
    const std_srvs::srv::Empty::Request::ConstSharedPtr &,
    const std_srvs::srv::Empty::Response::SharedPtr &);

  void publishWaypoints();
  void saveWaypoints(const nav_waypoint_msgs::msg::Waypoints &);
  nav_waypoint_msgs::msg::Waypoints loadWaypoints(const std::string & file_path);
  void tryLoadWaypointsFromFile(const std::string & file_path);
  bool isLoadableWaypoints(const std::string & file_path);
};

NavWaypointServerNode::NavWaypointServerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_server",
    rclcpp::NodeOptions(node_options).use_intra_process_comms(false)),
  waypoints_(),
  waypoints_publisher_(nullptr),
  regist_waypoint_subscriber_(nullptr),
  modify_waypoint_subscriber_(nullptr),
  save_latest_waypoint_service_(nullptr),
  publish_waypoints_service_(nullptr),
  param_listener_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());

  param_listener_ = std::make_unique<ParamListener>(
    this->get_node_parameters_interface()
  );
  params_ = param_listener_->get_params();
  tryLoadWaypointsFromFile(params_.waypoints_file);

  waypoints_.header.stamp = this->get_clock()->now();
  waypoints_.header.frame_id = params_.waypoint_frame_id;

  waypoints_publisher_ = this->create_publisher<nav_waypoint_msgs::msg::Waypoints>(
    "~/waypoints",
    rclcpp::QoS(5).transient_local()
  );
  regist_waypoint_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoint>(
    "~/regist",
    rclcpp::QoS(5),
    std::bind(
      &NavWaypointServerNode::registWaypointCallback,
      this,
      std::placeholders::_1
    )
  );
  modify_waypoint_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoint>(
    "~/modify",
    rclcpp::QoS(5),
    std::bind(
      &NavWaypointServerNode::modifyWaypointCallback,
      this,
      std::placeholders::_1
    )
  );
  save_latest_waypoint_service_ = this->create_service<std_srvs::srv::Empty>(
    "~/save",
    std::bind(
      &NavWaypointServerNode::saveWaypointsServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  publish_waypoints_service_ = this->create_service<std_srvs::srv::Empty>(
    "~/publish",
    std::bind(
      &NavWaypointServerNode::publishWaypointsServiceCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize successfully " << this->get_name());
  //! Publish loaded waypoints
  publishWaypoints();
}

NavWaypointServerNode::~NavWaypointServerNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void NavWaypointServerNode::registWaypointCallback(
  const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr & msg)
{
  bool overwrited = false;
  RCLCPP_INFO(this->get_logger(), "Recived new waypoint");

  for (auto && waypoint : waypoints_.waypoints) {
    if (waypoint.name == msg->name) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Overwrite waypoint: " << msg->name);
      waypoint = *msg;
      overwrited = true;
      break;
    }
  }
  if (!overwrited) {
    waypoints_.waypoints.push_back(*msg);
  }
  waypoints_.header.stamp = this->get_clock()->now();
  publishWaypoints();
  RCLCPP_INFO_STREAM(this->get_logger(), "Registed new waypoint: " << msg->name);
}

void NavWaypointServerNode::modifyWaypointCallback(
  const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr & msg)
{
  bool found_waypoint = false;
  for (auto && waypoint : waypoints_.waypoints) {
    if (waypoint.name != msg->name) {
      continue;
    }
    waypoint = *msg;
    found_waypoint = true;
    break;
  }
  if (!found_waypoint) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Ignore waypoint " << msg->name);
  }
}

void NavWaypointServerNode::saveWaypointsServiceCallback(
  const std_srvs::srv::Empty::Request::ConstSharedPtr &,
  const std_srvs::srv::Empty::Response::SharedPtr &)
{
  RCLCPP_INFO(this->get_logger(), "Start save waypoints");
  saveWaypoints(waypoints_);
  RCLCPP_INFO(this->get_logger(), "Save waypoints successfully");
}

void NavWaypointServerNode::publishWaypointsServiceCallback(
  const std_srvs::srv::Empty::Request::ConstSharedPtr &,
  const std_srvs::srv::Empty::Response::SharedPtr &)
{
  publishWaypoints();
}

void NavWaypointServerNode::publishWaypoints()
{
  nav_waypoint_msgs::msg::Waypoints::UniquePtr msg;
  msg = std::make_unique<nav_waypoint_msgs::msg::Waypoints>(waypoints_);
  waypoints_publisher_->publish(std::move(msg));
  RCLCPP_INFO_STREAM(this->get_logger(), "Publish waypoints");
}

void NavWaypointServerNode::saveWaypoints(const nav_waypoint_msgs::msg::Waypoints & waypoints)
{
  //! @todo refactoring
  YAML::Node waypoints_yaml_node{};

  for (const auto & waypoint : waypoints.waypoints) {
    YAML::Node local_yaml_node;
    YAML::Node pose_yaml_node;
    YAML::Node position_yaml_node;
    YAML::Node orientation_yaml_node;
    YAML::Node properties_yaml_node;

    position_yaml_node["x"] = waypoint.pose.position.x;
    position_yaml_node["y"] = waypoint.pose.position.y;
    position_yaml_node["z"] = waypoint.pose.position.z;
    pose_yaml_node["position"] = position_yaml_node;
    orientation_yaml_node["x"] = waypoint.pose.orientation.x;
    orientation_yaml_node["y"] = waypoint.pose.orientation.y;
    orientation_yaml_node["z"] = waypoint.pose.orientation.z;
    orientation_yaml_node["w"] = waypoint.pose.orientation.w;
    pose_yaml_node["orientation"] = orientation_yaml_node;
    local_yaml_node["pose"] = pose_yaml_node;

    for (const auto & property : waypoint.properties) {
      properties_yaml_node[property.key] = property.value;
    }
    local_yaml_node["properties"] = properties_yaml_node;

    waypoints_yaml_node[waypoint.name] = local_yaml_node;
  }
  YAML::Node waypoint_manager_yaml_node;
  YAML::Emitter yaml_emitter;

  waypoint_manager_yaml_node["waypoint_manager"]["waypoints"] = waypoints_yaml_node;
  yaml_emitter << waypoint_manager_yaml_node;

  std::ofstream waypoints_file;
  waypoints_file.open(params_.waypoints_file);

  if (!waypoints_file.is_open()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Can't open file " << params_.waypoints_file);
    return;
  }
  waypoints_file << yaml_emitter.c_str();
  waypoints_file.close();
}

nav_waypoint_msgs::msg::Waypoints NavWaypointServerNode::loadWaypoints(
  const std::string & file_path)
{
  nav_waypoint_msgs::msg::Waypoints ret_waypoints{};

  //! @todo refactoring
  YAML::Node yaml_node = YAML::LoadFile(file_path);
  YAML::Node waypoint_manager_yaml_node = yaml_node["waypoint_manager"];
  YAML::Node waypoints_yaml_node = waypoint_manager_yaml_node["waypoints"];

  for (auto itr = waypoints_yaml_node.begin(); itr != waypoints_yaml_node.end(); ++itr) {
    nav_waypoint_msgs::msg::Waypoint w;
    w.name = itr->first.as<std::string>();
    w.pose.position.x = itr->second["pose"]["position"]["x"].as<double>();
    w.pose.position.y = itr->second["pose"]["position"]["y"].as<double>();
    w.pose.position.z = itr->second["pose"]["position"]["z"].as<double>();
    w.pose.orientation.x = itr->second["pose"]["orientation"]["x"].as<double>();
    w.pose.orientation.y = itr->second["pose"]["orientation"]["y"].as<double>();
    w.pose.orientation.z = itr->second["pose"]["orientation"]["z"].as<double>();
    w.pose.orientation.w = itr->second["pose"]["orientation"]["w"].as<double>();
    const auto pbegin = itr->second["properties"].begin();
    const auto pend = itr->second["properties"].end();
    for (auto pitr = pbegin; pitr != pend; ++pitr) {
      nav_waypoint_msgs::msg::Property p;
      p.key = pitr->first.as<std::string>();
      p.value = pitr->second.as<std::string>();
      w.properties.push_back(p);
    }
    ret_waypoints.waypoints.push_back(w);
  }
  return ret_waypoints;
}

void NavWaypointServerNode::tryLoadWaypointsFromFile(const std::string & file_path)
{
  if (isLoadableWaypoints(file_path)) {
    waypoints_ = loadWaypoints(file_path);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Can't open waypoints file: " << file_path);
  }
}

bool NavWaypointServerNode::isLoadableWaypoints(const std::string & file_path)
{
  std::ifstream file;
  file.open(file_path);
  const bool is_open = file.is_open();
  file.close();
  return is_open;
}
}  // namespace nav_waypoint_server

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_server::NavWaypointServerNode)
