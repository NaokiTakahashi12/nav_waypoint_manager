#ifndef NAV_WAYPOINT_INTERACTIVE_MARKER_NODE_HPP_
#define NAV_WAYPOINT_INTERACTIVE_MARKER_NODE_HPP_

#include <nav_waypoint_msgs/srv/update_waypoint_pose.hpp>
#include <chrono>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <cmath>
#include <nav_waypoint_msgs/validation.hpp>
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
  void subscribeWaypointsCallback(const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr &);
  void feedbackWaypointCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &);
  void setArrowMarker(visualization_msgs::msg::InteractiveMarkerControl &);
  void setFlagMarkerFromProperties(
    visualization_msgs::msg::InteractiveMarkerControl &,
    const std::vector<nav_waypoint_msgs::msg::Property> &);

  nav_waypoint_interactive_marker_node::ParamListener param_listener_;
  nav_waypoint_interactive_marker_node::Params params_;

  rclcpp::Client<nav_waypoint_msgs::srv::UpdateWaypointPose>::SharedPtr pose_client_;
  rclcpp::TimerBase::SharedPtr request_timer_;
  int64_t pending_id_{0};
  bool pose_pending_{false};
  std::string pending_name_;
  std::chrono::steady_clock::time_point deadline_;
  void restorePose(const std::string & name, const std::string & message = "");
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoints>::SharedPtr waypoints_subscriber_;

  std_msgs::msg::Header waypoint_header_;
  geometry_msgs::msg::Pose displayPose(const geometry_msgs::msg::Pose & pose) const;
  std::unordered_map<std::string, nav_waypoint_msgs::msg::Waypoint> waypoint_index_;
  struct DragStart
  {
    std::string client_id;
    geometry_msgs::msg::Pose pose;
  };
  std::unordered_map<std::string, DragStart> drag_starts_;

  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
};
}  // namespace nav_waypoint_visualization
#endif  // NAV_WAYPOINT_INTERACTIVE_MARKER_NODE_HPP_
