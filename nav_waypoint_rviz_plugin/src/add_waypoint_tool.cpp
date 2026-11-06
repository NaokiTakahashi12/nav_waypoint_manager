// Copyright (c) 2023 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "nav_waypoint_rviz_plugin/add_waypoint_tool.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
namespace nav_waypoint_rviz_plugin
{
AddWaypointTool::AddWaypointTool()
{
  shortcut_key_ = 'w';
  topic_ = new rviz_common::properties::StringProperty(
    "Waypoint Topic", "/set_waypoint",
    "Registration input topic; match the Pose-to-Waypoint converter", getPropertyContainer());
  connect(topic_, &rviz_common::properties::Property::changed, this, [this]() {updatePublisher();});
}
void AddWaypointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Add Waypoint");
  initializeRos(context_->getRosNodeAbstraction().lock()->get_raw_node());
}
void AddWaypointTool::initializeRos(rclcpp::Node::SharedPtr node)
{
  node_ = std::move(node);
  updatePublisher();
}
void AddWaypointTool::updatePublisher()
{
  if (!node_) {return;}
  publisher_.reset();
  try {
    publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_->getStdString(), 2);
  } catch (const std::exception & e) {
    setStatus(QString::fromUtf8(e.what()));
  }
}
void AddWaypointTool::onPoseSet(double x, double y, double theta)
{
  publishPose(x, y, theta, context_->getFixedFrame().toStdString());
}
void AddWaypointTool::publishPose(double x, double y, double theta, const std::string & frame)
{
  if (!publisher_) {setStatus("Invalid waypoint registration topic"); return;}
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = orientationAroundZAxis(theta);
  publisher_->publish(pose);
}
}  // namespace nav_waypoint_rviz_plugin
PLUGINLIB_EXPORT_CLASS(
  nav_waypoint_rviz_plugin::AddWaypointTool,
  rviz_common::Tool)
