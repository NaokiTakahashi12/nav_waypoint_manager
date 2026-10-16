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

#include <chrono>
#include <string>
#include <stdexcept>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>

namespace nav_waypoint_conversion
{
class PoseStampedToNavWaypointNode : public rclcpp::Node
{
public:
  explicit PoseStampedToNavWaypointNode(const rclcpp::NodeOptions &);
  ~PoseStampedToNavWaypointNode();

private:
  std::string frame_id_;
  uint64_t callback_counter_{0};
  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoint>::SharedPtr waypoint_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  void poseSubscribeCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &);
};

PoseStampedToNavWaypointNode::PoseStampedToNavWaypointNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("pose_stamped_to_nav_waypoint",
    rclcpp::NodeOptions(node_options).use_intra_process_comms(true)),
  waypoint_publisher_(nullptr),
  pose_subscription_(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  frame_id_ = declare_parameter<std::string>("waypoint_frame_id", "map", descriptor);
  if (frame_id_.empty()) {
    throw std::invalid_argument("waypoint_frame_id must not be empty");
  }

  waypoint_publisher_ = this->create_publisher<nav_waypoint_msgs::msg::Waypoint>(
    "~/output_waypoint",
    rclcpp::QoS(2)
  );
  pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/input_pose",
    rclcpp::QoS(2),
    std::bind(
      &PoseStampedToNavWaypointNode::poseSubscribeCallback,
      this,
      std::placeholders::_1
    )
  );

  RCLCPP_INFO_STREAM(this->get_logger(), "Initialize successfully " << this->get_name());
}

PoseStampedToNavWaypointNode::~PoseStampedToNavWaypointNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void PoseStampedToNavWaypointNode::poseSubscribeCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  nav_waypoint_msgs::msg::Waypoint::UniquePtr waypoint_msg;

  if (pose_msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(this->get_logger(), "Input pose frame does not match waypoint_frame_id");
    return;
  }

  waypoint_msg = std::make_unique<nav_waypoint_msgs::msg::Waypoint>();
  waypoint_msg->frame_id = frame_id_;
  waypoint_msg->pose = pose_msg->pose;
  waypoint_msg->stamp = pose_msg->header.stamp;
  waypoint_msg->name =
    "wp_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
    "_" + std::to_string(callback_counter_++);
  {
    nav_waypoint_msgs::msg::Property default_waypoint_property;
    default_waypoint_property.key = "goal_reached_radius";
    default_waypoint_property.value = "1.0";
    waypoint_msg->properties.push_back(default_waypoint_property);
  }
  {
    nav_waypoint_msgs::msg::Property default_waypoint_property;
    default_waypoint_property.key = "stop_with_resume";
    default_waypoint_property.value = "false";
    waypoint_msg->properties.push_back(default_waypoint_property);
  }

  try {
    nav_waypoint_msgs::validateWaypoint(*waypoint_msg, frame_id_);
    waypoint_publisher_->publish(std::move(waypoint_msg));
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Rejected input pose: %s", e.what());
  }
}
}  // namespace nav_waypoint_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_conversion::PoseStampedToNavWaypointNode)
