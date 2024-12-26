#include <string>
#include <memory>
#include <functional>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_waypoint_msgs/msg/waypoints.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nav_waypoint_visualization
{
class NavWaypointVisualizationMarkerNode : public rclcpp::Node
{
public:
  NavWaypointVisualizationMarkerNode(const rclcpp::NodeOptions &);

private:
  void publishRouteMarker();
  void subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr &);
  void subscribeRouteCallback(const nav_waypoint_msgs::msg::Route::ConstSharedPtr &);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoints_subscriber_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Route>::SharedPtr route_subscriber_;

  nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr waypoints_;
  nav_waypoint_msgs::msg::Route::ConstSharedPtr route_;
};

NavWaypointVisualizationMarkerNode::NavWaypointVisualizationMarkerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_visualization_maker", node_options),
  route_marker_publisher_(nullptr),
  waypoints_subscriber_(nullptr),
  route_subscriber_(nullptr)
{
  RCLCPP_INFO(this->get_logger(), "Start initialize");

  route_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/route_marker",
    rclcpp::QoS(1).transient_local());

  waypoints_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoints>(
    "nav_waypoint_server/waypoints",
    rclcpp::QoS(1).transient_local(),
    std::bind(
      &NavWaypointVisualizationMarkerNode::subscribeWaypointsCallback,
      this,
      std::placeholders::_1));

  route_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Route>(
    "nav_route_server/route",
    rclcpp::QoS(1).transient_local(),
    std::bind(
      &NavWaypointVisualizationMarkerNode::subscribeRouteCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialize successful");
}

void NavWaypointVisualizationMarkerNode::publishRouteMarker()
{
  if (!route_ || !waypoints_) {
    return;
  }
  std::vector<unsigned int> matched_waypoint_indexes;

  for (const auto & waypoint_name_on_route : route_->route) {
    unsigned int matched_waypoint_index = 0;
    for (const auto & waypoint : waypoints_->waypoints) {
      if (waypoint_name_on_route == waypoint.name) {
        break;
      }
      matched_waypoint_index++;
    }
    if (matched_waypoint_index >= waypoints_->waypoints.size()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Skipping routed waypoint name: " << waypoint_name_on_route);
      continue;
    }
    matched_waypoint_indexes.push_back(matched_waypoint_index);
  }
  if (matched_waypoint_indexes.size() < 1) {
    RCLCPP_INFO(this->get_logger(), "Empty visualize waypoint route");
    return;
  }
  auto route_marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  for (const auto & waypoint_index : matched_waypoint_indexes) {
    visualization_msgs::msg::Marker waypoint_name_merker_msg;
    waypoint_name_merker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    waypoint_name_merker_msg.text = waypoints_->waypoints[waypoint_index].name;
    waypoint_name_merker_msg.pose.position = waypoints_->waypoints[waypoint_index].pose.position;
    waypoint_name_merker_msg.pose.position.z += 1.5;
    waypoint_name_merker_msg.color.r = 1.0;
    waypoint_name_merker_msg.color.g = 1.0;
    waypoint_name_merker_msg.color.b = 1.0;
    waypoint_name_merker_msg.color.a = 1.0;
    waypoint_name_merker_msg.scale.z = 0.5;
    route_marker_array_msg->markers.push_back(waypoint_name_merker_msg);
  }

  visualization_msgs::msg::Marker route_line_marker_msg;
  route_line_marker_msg.points.resize(matched_waypoint_indexes.size());
  int route_index = 0;

  for (const auto & waypoint_index : matched_waypoint_indexes) {
    route_line_marker_msg.points[route_index] = waypoints_->waypoints[waypoint_index].pose.position;
    route_index++;
  }
  route_line_marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  route_line_marker_msg.scale.x = 0.5;
  route_line_marker_msg.color.r = 0.0;
  route_line_marker_msg.color.g = 1.0;
  route_line_marker_msg.color.b = 0.5;
  route_line_marker_msg.color.a = 0.7;
  route_marker_array_msg->markers.push_back(route_line_marker_msg);

  int marker_id = 0;
  for (auto && m : route_marker_array_msg->markers) {
    m.ns = "route_waypoint";
    m.header = waypoints_->header;
    m.id = marker_id;
    marker_id++;
  }
  route_marker_publisher_->publish(std::move(route_marker_array_msg));
}

void NavWaypointVisualizationMarkerNode::subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr & msg)
{
  if (msg->waypoints.size() < 1) {
    RCLCPP_INFO(this->get_logger(), "Empty waypoints msg subscribed");
    return;
  }
  waypoints_ = msg;

  if (route_ && waypoints_) {
    publishRouteMarker();
  }
}

void NavWaypointVisualizationMarkerNode::subscribeRouteCallback(const nav_waypoint_msgs::msg::Route::ConstSharedPtr & msg)
{
  if (msg->route.size() < 1) {
    RCLCPP_INFO(this->get_logger(), "Empty route msg subscribed");
    return;
  }
  route_ = msg;

  if (waypoints_ && route_) {
    publishRouteMarker();
  }
}
}  // namespace nav_waypoint_visualization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_visualization::NavWaypointVisualizationMarkerNode)
