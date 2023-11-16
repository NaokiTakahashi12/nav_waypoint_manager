#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_waypoint_msgs/msg/waypoint.hpp>

namespace nav_waypoint_conversion
{
class PoseStampedToNavWaypointNode : public rclcpp::Node
{
public:
  PoseStampedToNavWaypointNode(const rclcpp::NodeOptions &);
  ~PoseStampedToNavWaypointNode();

private:
  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoint>::SharedPtr m_waypoint_publisher;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_subscription;

  void poseSubscribeCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &);
};

PoseStampedToNavWaypointNode::PoseStampedToNavWaypointNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("pose_stamped_to_nav_waypoint", node_options),
  m_waypoint_publisher(nullptr),
  m_pose_subscription(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start initialize " << this->get_name());

  m_waypoint_publisher = this->create_publisher<nav_waypoint_msgs::msg::Waypoint>(
    "~/output_waypoint",
    rclcpp::QoS(2)
  );
  m_pose_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
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

  //! @todo to parameter
  if (pose_msg->header.frame_id != "map") {
    RCLCPP_INFO(this->get_logger(), "Please input map frame");
    return;
  }

  waypoint_msg = std::make_unique<nav_waypoint_msgs::msg::Waypoint>();
  waypoint_msg->pose = pose_msg->pose;
  waypoint_msg->stamp = pose_msg->header.stamp;

  m_waypoint_publisher->publish(std::move(waypoint_msg));
}
}  // namespace nav_waypoint_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_conversion::PoseStampedToNavWaypointNode)
