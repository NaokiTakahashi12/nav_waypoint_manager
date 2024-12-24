#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <memory>
#include <functional>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_waypoint_msgs/msg/waypoints.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>
#include <nav_waypoint_msgs/msg/property.hpp>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include "nav_waypoint_interactive_marker_node_params.hpp"

namespace nav_waypoint_visualization
{
class NavWaypointInteractiveMarkerNode : public rclcpp::Node
{
public:
  NavWaypointInteractiveMarkerNode(const rclcpp::NodeOptions &);

private:
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_marker_publisher_;
  void subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr &);
  void feedbackWaypointCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &);
  void setArrowMarker(visualization_msgs::msg::InteractiveMarkerControl &);
  void setFlagMarkerFromProperties(
    visualization_msgs::msg::InteractiveMarkerControl &,
    const std::vector<nav_waypoint_msgs::msg::Property> &);

  nav_waypoint_interactive_marker_node::ParamListener param_listener_;
  nav_waypoint_interactive_marker_node::Params params_;

  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoint>::SharedPtr modify_waypoint_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoints_subscriber_;

  nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr last_waypoints_;

  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
};

NavWaypointInteractiveMarkerNode::NavWaypointInteractiveMarkerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_interactive_marker", node_options),
  param_listener_(this->get_node_parameters_interface()),
  params_(param_listener_.get_params()),
  modify_waypoint_publisher_(nullptr),
  waypoints_subscriber_(nullptr),
  last_waypoints_(nullptr),
  interactive_marker_server_("~/interactive_marker", this)
{
  RCLCPP_INFO(this->get_logger(), "Start initialize");

  modify_waypoint_publisher_ = this->create_publisher<nav_waypoint_msgs::msg::Waypoint>(
    "nav_waypoint_server/modify",
    rclcpp::QoS(3));

  waypoints_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoints>(
    "nav_waypoint_server/waypoints",
    rclcpp::QoS(1).transient_local(),
    std::bind(
      &NavWaypointInteractiveMarkerNode::subscribeWaypointsCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialize successful");
}

void NavWaypointInteractiveMarkerNode::subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr & msg)
{
  if (msg->waypoints.size() <= 0) {
    return;
  }
  interactive_marker_server_.clear();

  for (const auto & waypoint : msg->waypoints) {
    visualization_msgs::msg::InteractiveMarker interactive_marker_msg;
    visualization_msgs::msg::InteractiveMarkerControl waypoint_control;
    visualization_msgs::msg::InteractiveMarkerControl waypoint_orientation_control;

    waypoint_control.always_visible = true;
    waypoint_orientation_control.always_visible = true;

    interactive_marker_msg.pose = waypoint.pose;

    if (params_.interactive_xy_only) {
      waypoint_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
      waypoint_orientation_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      interactive_marker_msg.pose.position.z = 0.0;
      interactive_marker_msg.pose.orientation.x = 0.0;
      interactive_marker_msg.pose.orientation.y = 0.0;
    } else {
      waypoint_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
      waypoint_orientation_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
    }
    waypoint_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    waypoint_orientation_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;

    tf2::Quaternion orientation(0, 1, 0, 1);
    orientation.normalize();
    tf2::convert(orientation, waypoint_control.orientation);
    tf2::convert(orientation, waypoint_orientation_control.orientation);

    setArrowMarker(waypoint_orientation_control);
    setFlagMarkerFromProperties(waypoint_control, waypoint.properties);

    interactive_marker_msg.controls.push_back(waypoint_control);
    interactive_marker_msg.controls.push_back(waypoint_orientation_control);
    interactive_marker_msg.header.frame_id = msg->header.frame_id;
    interactive_marker_msg.header.stamp = this->get_clock()->now();
    interactive_marker_msg.name = waypoint.name;
    interactive_marker_msg.description = waypoint.name;

    interactive_marker_server_.insert(
      interactive_marker_msg,
      std::bind(
        &NavWaypointInteractiveMarkerNode::feedbackWaypointCallback,
        this,
        std::placeholders::_1));
  }
  interactive_marker_server_.applyChanges();
  last_waypoints_ = msg;
}

void NavWaypointInteractiveMarkerNode::feedbackWaypointCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  static bool last_mouse_point_valid = true;

  if (feedback->mouse_point_valid && !last_mouse_point_valid) {
    auto msg = std::make_unique<nav_waypoint_msgs::msg::Waypoint>();

    msg->stamp = this->get_clock()->now();
    msg->name = feedback->marker_name;
    msg->pose = feedback->pose;

    for (const auto & lw : last_waypoints_->waypoints) {
      if (msg->name == lw.name) {
        msg->properties = lw.properties;
        break;
      }
    }
    modify_waypoint_publisher_->publish(std::move(msg));
    interactive_marker_server_.setPose(feedback->marker_name, feedback->pose);
    interactive_marker_server_.applyChanges();
  }

  last_mouse_point_valid = feedback->mouse_point_valid;
}

void NavWaypointInteractiveMarkerNode::setArrowMarker(
  visualization_msgs::msg::InteractiveMarkerControl & waypoint_orientation_control)
{
  visualization_msgs::msg::Marker arrow_marker;

  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.scale.x = 1.0;
  arrow_marker.scale.y = 0.05;
  arrow_marker.scale.z = 0.3;
  arrow_marker.pose.position.x = 0.0;
  arrow_marker.pose.position.y = 0.0;
  arrow_marker.pose.position.z = 1.0;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 0.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.a = 0.6;

  waypoint_orientation_control.markers.push_back(arrow_marker);
}

void NavWaypointInteractiveMarkerNode::setFlagMarkerFromProperties(
  visualization_msgs::msg::InteractiveMarkerControl & waypoint_control,
  const std::vector<nav_waypoint_msgs::msg::Property> & properties)
{
  double goal_reached_radius = 1.0;

  for (const auto & p : properties) {
    if ("goal_reached_radius" == p.key) {
      goal_reached_radius = std::stof(p.value);
      break;
    }
  }

  std::vector<visualization_msgs::msg::Marker> flag_marker_parts(4);

  flag_marker_parts[0].type = visualization_msgs::msg::Marker::CYLINDER;
  flag_marker_parts[0].scale.x = 0.1;
  flag_marker_parts[0].scale.y = 0.1;
  flag_marker_parts[0].scale.z = 1.2;
  flag_marker_parts[0].pose.position.x = 0.0;
  flag_marker_parts[0].pose.position.y = 0.0;
  flag_marker_parts[0].pose.position.z = flag_marker_parts[0].scale.z * 0.5;
  flag_marker_parts[0].color.r = 0.75;
  flag_marker_parts[0].color.g = 0.75;
  flag_marker_parts[0].color.b = 0.75;
  flag_marker_parts[0].color.a = 0.6;

  flag_marker_parts[1].type = visualization_msgs::msg::Marker::SPHERE;
  flag_marker_parts[1].scale.x = 0.2;
  flag_marker_parts[1].scale.y = 0.2;
  flag_marker_parts[1].scale.z = 0.2;
  flag_marker_parts[1].pose.position.x = 0.0;
  flag_marker_parts[1].pose.position.y = 0.0;
  flag_marker_parts[1].pose.position.z = 1.25;
  flag_marker_parts[1].color.r = 1.0;
  flag_marker_parts[1].color.g = 1.0;
  flag_marker_parts[1].color.b = 0.0;
  flag_marker_parts[1].color.a = 0.6;

  flag_marker_parts[2].type = visualization_msgs::msg::Marker::CYLINDER;
  flag_marker_parts[2].scale.x = 0.9;
  flag_marker_parts[2].scale.y = 0.9;
  flag_marker_parts[2].scale.z = 1.5;
  flag_marker_parts[2].pose.position.x = 0.0;
  flag_marker_parts[2].pose.position.y = 0.0;
  flag_marker_parts[2].pose.position.z = flag_marker_parts[2].scale.z * 0.5;
  flag_marker_parts[2].color.a = 0.0;

  flag_marker_parts[3].type = visualization_msgs::msg::Marker::CYLINDER;
  flag_marker_parts[3].scale.x = 2.0 * goal_reached_radius;
  flag_marker_parts[3].scale.y = 2.0 * goal_reached_radius;
  flag_marker_parts[3].scale.z = 0.03;
  flag_marker_parts[3].color.r = 0.0;
  flag_marker_parts[3].color.g = 1.0;
  flag_marker_parts[3].color.b = 0.0;
  flag_marker_parts[3].color.a = 0.3;

  for (const auto & marker : flag_marker_parts) {
    waypoint_control.markers.push_back(marker);
  }
}
}  // namespace nav_waypoint_visualization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_visualization::NavWaypointInteractiveMarkerNode)
