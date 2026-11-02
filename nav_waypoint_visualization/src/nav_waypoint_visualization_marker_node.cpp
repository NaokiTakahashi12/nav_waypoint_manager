#include <string>
#include <memory>
#include <functional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_waypoint_msgs/msg/waypoints.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <nav_waypoint_msgs/msg/waypoint_route_configuration.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nav_waypoint_visualization
{
class NavWaypointVisualizationMarkerNode : public rclcpp::Node
{
public:
  NavWaypointVisualizationMarkerNode(const rclcpp::NodeOptions &);

private:
  void publishRouteMarker();
  visualization_msgs::msg::MarkerArray last_rendered_;
  bool rendered_{false};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::WaypointRouteConfiguration>::SharedPtr
    configuration_subscriber_;

  std::vector<nav_waypoint_msgs::msg::Waypoint> route_waypoints_;
  std::string route_frame_id_;
};

NavWaypointVisualizationMarkerNode::NavWaypointVisualizationMarkerNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_visualization_maker", node_options),
  route_marker_publisher_(nullptr)
{
  RCLCPP_INFO(this->get_logger(), "Start initialize");

  route_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/route_marker",
    rclcpp::QoS(1).transient_local());

  configuration_subscriber_ =
    create_subscription<nav_waypoint_msgs::msg::WaypointRouteConfiguration>(
    "nav_waypoint_server/configuration", rclcpp::QoS(1).transient_local(),
    [this](nav_waypoint_msgs::msg::WaypointRouteConfiguration::ConstSharedPtr msg) {
      route_frame_id_ = msg->registered_waypoints.header.frame_id;
      try {
        route_waypoints_ = nav_waypoint_msgs::resolveRouteWaypoints(*msg, route_frame_id_);
      } catch (const std::exception & e) {
        route_waypoints_.clear();
        RCLCPP_WARN(get_logger(), "Invalid waypoint route configuration: %s", e.what());
      }
      publishRouteMarker();
    });

  RCLCPP_INFO(this->get_logger(), "Initialize successful");
}

void NavWaypointVisualizationMarkerNode::publishRouteMarker()
{
  auto route_marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  for (const auto & waypoint : route_waypoints_) {
    visualization_msgs::msg::Marker waypoint_name_merker_msg;
    waypoint_name_merker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    waypoint_name_merker_msg.text = waypoint.name;
    waypoint_name_merker_msg.pose.position = waypoint.pose.position;
    waypoint_name_merker_msg.pose.position.z += 1.5;
    waypoint_name_merker_msg.color.r = 1.0;
    waypoint_name_merker_msg.color.g = 1.0;
    waypoint_name_merker_msg.color.b = 1.0;
    waypoint_name_merker_msg.color.a = 1.0;
    waypoint_name_merker_msg.scale.z = 0.5;
    route_marker_array_msg->markers.push_back(waypoint_name_merker_msg);
  }

  visualization_msgs::msg::Marker route_line_marker_msg;
  route_line_marker_msg.points.resize(route_waypoints_.size());
  int route_index = 0;

  for (const auto & waypoint : route_waypoints_) {
    route_line_marker_msg.points[route_index] = waypoint.pose.position;
    route_index++;
  }
  route_line_marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  route_line_marker_msg.scale.x = 0.5;
  route_line_marker_msg.color.r = 0.0;
  route_line_marker_msg.color.g = 1.0;
  route_line_marker_msg.color.b = 0.5;
  route_line_marker_msg.color.a = 0.7;
  if (route_waypoints_.size() >= 2) {
    route_marker_array_msg->markers.push_back(route_line_marker_msg);
  }

  int marker_id = 0;
  for (auto & m : route_marker_array_msg->markers) {
    m.ns = "route_waypoint";
    m.header.frame_id = route_frame_id_;
    m.id = marker_id++;
  }
  if (rendered_ && *route_marker_array_msg == last_rendered_) {return;}
  const auto previous_size = last_rendered_.markers.size();
  last_rendered_ = *route_marker_array_msg;
  rendered_ = true;
  // Keep a complete latched snapshot for RViz clients that join later.
  // Delete only obsolete IDs instead of rebuilding everything with DELETEALL.
  for (std::size_t i = route_marker_array_msg->markers.size(); i < previous_size; ++i) {
    visualization_msgs::msg::Marker removed;
    removed.ns = "route_waypoint";
    removed.id = static_cast<int>(i);
    removed.action = visualization_msgs::msg::Marker::DELETE;
    removed.header.frame_id = route_frame_id_;
    route_marker_array_msg->markers.push_back(removed);
  }
  route_marker_publisher_->publish(std::move(route_marker_array_msg));
}

}  // namespace nav_waypoint_visualization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_visualization::NavWaypointVisualizationMarkerNode)
