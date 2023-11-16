#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>

namespace nav_waypoint_conversion
{
class NavWaypointToPoseStampedNode : public rclcpp::Node
{
public:
  NavWaypointToPoseStampedNode(const rclcpp::NodeOptions &);
  ~NavWaypointToPoseStampedNode();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoint>::SharedPtr m_waypoint_subscription;

  void waypoitSubscribeCallback(const nav_waypoint_msgs::msg::Waypoint::ConstSharedPtr &);
};

NavWaypointToPoseStampedNode::NavWaypointToPoseStampedNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_to_pose_stamped", node_options),
  m_pose_publisher(nullptr),
  m_waypoint_subscription(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());

  m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/output_pose",
    rclcpp::QoS(2)
  );
  m_waypoint_subscription = this->create_subscription<nav_waypoint_msgs::msg::Waypoint>(
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

  m_pose_publisher->publish(std::move(pose_msg));
}
}  // namespace nav_waypoint_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_conversion::NavWaypointToPoseStampedNode)
