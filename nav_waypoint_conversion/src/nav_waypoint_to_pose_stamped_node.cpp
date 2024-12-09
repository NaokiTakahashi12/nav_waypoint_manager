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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>

namespace nav_waypoint_conversion
{
class NavWaypointToPoseStampedNode : public rclcpp::Node
{
public:
  explicit NavWaypointToPoseStampedNode(const rclcpp::NodeOptions &);
  ~NavWaypointToPoseStampedNode();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoint>::SharedPtr waypoint_subscription_;

  void waypoitSubscribeCallback(const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr &);
};

NavWaypointToPoseStampedNode::NavWaypointToPoseStampedNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_to_pose_stamped",
    rclcpp::NodeOptions(node_options).use_intra_process_comms(true)),
  pose_publisher_(nullptr),
  waypoint_subscription_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/output_pose",
    rclcpp::QoS(2)
  );
  waypoint_subscription_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoint>(
    "~/input_waypoint",
    rclcpp::QoS(5),
    std::bind(
      &NavWaypointToPoseStampedNode::waypoitSubscribeCallback,
      this,
      std::placeholders::_1
    )
  );
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize successfully " << this->get_name());
}

NavWaypointToPoseStampedNode::~NavWaypointToPoseStampedNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void NavWaypointToPoseStampedNode::waypoitSubscribeCallback(
  const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr & waypoint_msg)
{
  geometry_msgs::msg::PoseStamped::UniquePtr pose_msg;

  pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  pose_msg->pose = waypoint_msg->pose;
  pose_msg->header.stamp = waypoint_msg->stamp;
  pose_msg->header.frame_id = "map";

  pose_publisher_->publish(std::move(pose_msg));
}
}  // namespace nav_waypoint_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_conversion::NavWaypointToPoseStampedNode)
