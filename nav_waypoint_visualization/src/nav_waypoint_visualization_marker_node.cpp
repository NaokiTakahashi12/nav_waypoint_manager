#include <string>
#include <memory>
#include <functional>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_waypoint_msgs/msg/waypoints.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nav_waypoint_visualization
{
class NavWaypointVisualizationMarkerNode : public rclcpp::Node
{
public:
  NavWaypointVisualizationMarkerNode(const rclcpp::NodeOptions &);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_marker_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoints_subscriber_;

  void subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr &);
};

NavWaypointVisualizationMarkerNode::NavWaypointVisualizationMarkerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_visualization_maker", node_options),
  waypoints_marker_publisher_(nullptr),
  waypoints_subscriber_(nullptr)
{
  waypoints_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/waypoints_marker_array",
    rclcpp::QoS(1).transient_local());

  waypoints_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoints>(
    "nav_waypoint_server/waypoints",
    rclcpp::QoS(1).transient_local(),
    std::bind(
      &NavWaypointVisualizationMarkerNode::subscribeWaypointsCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialize successful");
}

void NavWaypointVisualizationMarkerNode::subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr & msg)
{
  if (msg->waypoints.size() <= 0) {
    RCLCPP_INFO(this->get_logger(), "Empty waypoints msg subscribed");
    return;
  }
  auto markers_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  int marker_id = 0;
  for (const auto & waypoint : msg->waypoints) {
    visualization_msgs::msg::Marker marker_msg;

    float goal_reached_radius = 1.0;
    for (const auto & property : waypoint.properties) {
      if (property.key == "goal_reached_radius") {
        goal_reached_radius = std::stof(property.value);
        break;
      }
    }

    marker_msg.header = msg->header;
    marker_msg.pose = waypoint.pose;
    marker_msg.ns = "waypoints";
    marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = goal_reached_radius;
    marker_msg.scale.y = goal_reached_radius;
    marker_msg.scale.z = 0.2;
    marker_msg.color.r = 0.5;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.5;
    marker_msg.color.a = 0.5;

    marker_msg.id = marker_id;

    markers_msg->markers.push_back(marker_msg);
    marker_id++;
  }
  for (const auto & waypoint : msg->waypoints) {
    visualization_msgs::msg::Marker marker_msg;

    marker_msg.header = msg->header;
    marker_msg.text = waypoint.name;
    marker_msg.pose = waypoint.pose;
    marker_msg.pose.position.z += 0.2;
    marker_msg.ns = "waypoints";
    marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = 0.5;
    marker_msg.scale.y = 0.5;
    marker_msg.scale.z = 0.5;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 1.0;

    marker_msg.id = marker_id;

    markers_msg->markers.push_back(marker_msg);
    marker_id++;
  }

  waypoints_marker_publisher_->publish(std::move(markers_msg));
}
}  // namespace nav_waypoint_visualization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_visualization::NavWaypointVisualizationMarkerNode)
