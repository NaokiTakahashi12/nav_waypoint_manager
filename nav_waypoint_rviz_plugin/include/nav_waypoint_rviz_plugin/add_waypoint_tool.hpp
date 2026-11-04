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

#ifndef NAV_WAYPOINT_RVIZ_PLUGIN__ADD_WAYPOINT_TOOL_HPP_
#define NAV_WAYPOINT_RVIZ_PLUGIN__ADD_WAYPOINT_TOOL_HPP_
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
namespace nav_waypoint_rviz_plugin
{
class AddWaypointTool : public rviz_default_plugins::tools::PoseTool
{
public:
  AddWaypointTool();
  void onInitialize() override;
  void initializeRos(rclcpp::Node::SharedPtr node);

protected:
  void onPoseSet(double x, double y, double theta) override;
  void publishPose(double x, double y, double theta, const std::string & frame);

private:
  rviz_common::properties::StringProperty * topic_;
  void updatePublisher();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};
}  // namespace nav_waypoint_rviz_plugin
#endif  // NAV_WAYPOINT_RVIZ_PLUGIN__ADD_WAYPOINT_TOOL_HPP_
