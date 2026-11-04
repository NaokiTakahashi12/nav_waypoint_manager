#include "nav_waypoint_interactive_marker_node.hpp"

namespace nav_waypoint_visualization
{


NavWaypointInteractiveMarkerNode::NavWaypointInteractiveMarkerNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("nav_waypoint_interactive_marker", node_options),
  param_listener_(this->get_node_parameters_interface()),
  params_(param_listener_.get_params()),
  waypoints_subscriber_(nullptr),
  interactive_marker_server_("~/interactive_marker", this)
{
  RCLCPP_INFO(this->get_logger(), "Start initialize");

  pose_client_ = create_client<nav_waypoint_msgs::srv::UpdateWaypointPose>(
    "nav_waypoint_server/update_pose");
  request_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() {
      if (pose_pending_ && std::chrono::steady_clock::now() >= deadline_) {
        pose_client_->remove_pending_request(pending_id_);
        pose_pending_ = false;
        restorePose(pending_name_, "Update timed out; result unknown");
        RCLCPP_WARN(get_logger(), "Pose update timed out; result is unknown");
      }
    });

  waypoints_subscriber_ = this->create_subscription<nav_waypoint_msgs::msg::Waypoints>(
    "nav_waypoint_server/waypoints",
    rclcpp::QoS(1).transient_local(),
    std::bind(
      &NavWaypointInteractiveMarkerNode::subscribeWaypointsCallback,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialize successful");
}

void NavWaypointInteractiveMarkerNode::subscribeWaypointsCallback(
  const nav_waypoint_msgs::msg::Waypoints::ConstSharedPtr & msg)
{
  try {
    nav_waypoint_msgs::validateWaypoints(*msg, msg->header.frame_id);
    if (msg->header.frame_id.empty()) {
      throw std::invalid_argument("Empty waypoint frame");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Invalid markers: %s", e.what());
    return;
  }
  std::unordered_map<std::string, nav_waypoint_msgs::msg::Waypoint> incoming;
  for (const auto & w : msg->waypoints) {
    incoming.emplace(w.name, w);
  }
  bool changed = false;
  for (const auto & old : waypoint_index_) {
    if (!incoming.count(old.first)) {
      interactive_marker_server_.erase(old.first);
      changed = true;
    }
  }
  for (const auto & waypoint : msg->waypoints) {
    const auto old = waypoint_index_.find(waypoint.name);
    if (!waypoint_header_.frame_id.empty() && waypoint_header_.frame_id == msg->header.frame_id &&
      old != waypoint_index_.end() && nav_waypoint_msgs::sameWaypoint(old->second, waypoint))
    {
      continue;
    }
    changed = true;
    visualization_msgs::msg::InteractiveMarker interactive_marker_msg;
    visualization_msgs::msg::InteractiveMarkerControl waypoint_control;
    visualization_msgs::msg::InteractiveMarkerControl waypoint_orientation_control;

    waypoint_control.always_visible = true;
    waypoint_orientation_control.always_visible = true;

    interactive_marker_msg.pose = displayPose(waypoint.pose);

    if (params_.interactive_xy_only) {
      waypoint_control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
      waypoint_orientation_control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    } else {
      waypoint_control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
      waypoint_orientation_control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
    }
    waypoint_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    waypoint_orientation_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::INHERIT;

    tf2::Quaternion orientation(0, 1, 0, 1);
    orientation.normalize();
    tf2::convert(orientation, waypoint_control.orientation);
    tf2::convert(orientation, waypoint_orientation_control.orientation);

    setArrowMarker(waypoint_orientation_control);
    setFlagMarkerFromProperties(waypoint_control, waypoint.properties);

    interactive_marker_msg.controls.push_back(waypoint_control);
    interactive_marker_msg.controls.push_back(waypoint_orientation_control);
    interactive_marker_msg.header.frame_id = msg->header.frame_id;
    interactive_marker_msg.header.stamp = this->get_clock()->now();
    interactive_marker_msg.name = waypoint.name;
    interactive_marker_msg.description = waypoint.name;

    interactive_marker_server_.insert(
      interactive_marker_msg,
      std::bind(
        &NavWaypointInteractiveMarkerNode::feedbackWaypointCallback,
        this,
        std::placeholders::_1));
  }
  if (changed) {interactive_marker_server_.applyChanges();}
  waypoint_index_ = std::move(incoming);
  waypoint_header_ = msg->header;
}

void NavWaypointInteractiveMarkerNode::feedbackWaypointCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (waypoint_header_.frame_id.empty() ||
    feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP ||
    feedback->header.frame_id != waypoint_header_.frame_id)
  {
    return;
  }
  const auto found = waypoint_index_.find(feedback->marker_name);
  if (found == waypoint_index_.end()) {return;}
  if (found->second.pose == feedback->pose) {return;}
  auto msg = std::make_unique<nav_waypoint_msgs::msg::Waypoint>(found->second);
  msg->stamp = now();
  msg->frame_id = waypoint_header_.frame_id;
  msg->pose = feedback->pose;
  try {
    nav_waypoint_msgs::validateWaypoint(*msg, msg->frame_id);
    if (params_.interactive_xy_only) {
      msg->pose.position.z = found->second.pose.position.z;
      tf2::Quaternion original, edited;
      tf2::fromMsg(found->second.pose.orientation, original);
      tf2::fromMsg(feedback->pose.orientation, edited);
      double roll, pitch, yaw, ignored_roll, ignored_pitch;
      tf2::Matrix3x3(original).getRPY(roll, pitch, yaw);
      tf2::Matrix3x3(edited).getRPY(ignored_roll, ignored_pitch, yaw);
      edited.setRPY(roll, pitch, yaw);
      msg->pose.orientation = tf2::toMsg(edited);
    }
    nav_waypoint_msgs::validateWaypoint(*msg, msg->frame_id);
    if (msg->pose == found->second.pose) {return;}
    if (pose_pending_ || !pose_client_->service_is_ready()) {
      restorePose(feedback->marker_name, "Update busy or unavailable");
      RCLCPP_WARN(get_logger(), "Pose update busy or service unavailable");
      return;
    }
    auto request = std::make_shared<nav_waypoint_msgs::srv::UpdateWaypointPose::Request>();
    request->name = msg->name;
    request->frame_id = msg->frame_id;
    request->pose = msg->pose;
    request->expected_pose = found->second.pose;
    pending_name_ = msg->name;
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    pending_id_ = pose_client_->async_send_request(
      request,
      [this,
      name = msg->name](
        rclcpp::Client<nav_waypoint_msgs::srv::UpdateWaypointPose>::SharedFuture f) {
        pose_pending_ = false;
        try {
          const auto response = f.get();
          if (!response->success) {
            restorePose(name, response->message);
            RCLCPP_WARN(get_logger(), "Pose update rejected: %s", response->message.c_str());
          }
        } catch (const std::exception & e) {
          restorePose(name, e.what());
          RCLCPP_WARN(get_logger(), "Pose update response failed: %s", e.what());
        }
      }).request_id;
    pose_pending_ = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Invalid marker pose: %s", e.what());
    restorePose(feedback->marker_name, e.what());
  }
  interactive_marker_server_.applyChanges();
}

geometry_msgs::msg::Pose NavWaypointInteractiveMarkerNode::displayPose(
  const geometry_msgs::msg::Pose & original) const
{
  auto pose = original;
  if (params_.interactive_xy_only) {
    pose.position.z = 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    q.setRPY(0.0, 0.0, yaw);
    pose.orientation = tf2::toMsg(q);
  }
  return pose;
}

void NavWaypointInteractiveMarkerNode::restorePose(
  const std::string & name, const std::string & message)
{
  const auto found = waypoint_index_.find(name);
  if (found == waypoint_index_.end()) {return;}
  const auto pose = displayPose(found->second.pose);
  visualization_msgs::msg::InteractiveMarker marker;
  if (!interactive_marker_server_.get(name, marker)) {return;}
  marker.pose = pose;
  marker.description = message.empty() ? name : name + ": " + message;
  interactive_marker_server_.insert(
    marker, std::bind(
      &NavWaypointInteractiveMarkerNode::feedbackWaypointCallback, this, std::placeholders::_1));
  interactive_marker_server_.applyChanges();
}

void NavWaypointInteractiveMarkerNode::setArrowMarker(
  visualization_msgs::msg::InteractiveMarkerControl & waypoint_orientation_control)
{
  visualization_msgs::msg::Marker arrow_marker;

  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.scale.x = 1.0;
  arrow_marker.scale.y = 0.05;
  arrow_marker.scale.z = 0.3;
  arrow_marker.pose.position.x = 0.0;
  arrow_marker.pose.position.y = 0.0;
  arrow_marker.pose.position.z = 1.0;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 0.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.a = 0.6;

  waypoint_orientation_control.markers.push_back(arrow_marker);
}

void NavWaypointInteractiveMarkerNode::setFlagMarkerFromProperties(
  visualization_msgs::msg::InteractiveMarkerControl & waypoint_control,
  const std::vector<nav_waypoint_msgs::msg::Property> & properties)
{
  nav_waypoint_msgs::msg::Waypoint waypoint;
  waypoint.properties = properties;
  const auto settings = nav_waypoint_msgs::reachSettings(waypoint);
  const double goal_reached_radius = settings.first;
  const bool stop_with_resume = settings.second;

  std::vector<visualization_msgs::msg::Marker> flag_marker_parts(4);

  flag_marker_parts[0].type = visualization_msgs::msg::Marker::CYLINDER;
  flag_marker_parts[0].scale.x = 0.1;
  flag_marker_parts[0].scale.y = 0.1;
  flag_marker_parts[0].scale.z = 1.2;
  flag_marker_parts[0].pose.position.x = 0.0;
  flag_marker_parts[0].pose.position.y = 0.0;
  flag_marker_parts[0].pose.position.z = flag_marker_parts[0].scale.z * 0.5;
  flag_marker_parts[0].color.r = 0.75;
  flag_marker_parts[0].color.g = 0.75;
  flag_marker_parts[0].color.b = 0.75;
  flag_marker_parts[0].color.a = 0.6;

  flag_marker_parts[1].type = visualization_msgs::msg::Marker::SPHERE;
  flag_marker_parts[1].scale.x = 0.2;
  flag_marker_parts[1].scale.y = 0.2;
  flag_marker_parts[1].scale.z = 0.2;
  flag_marker_parts[1].pose.position.x = 0.0;
  flag_marker_parts[1].pose.position.y = 0.0;
  flag_marker_parts[1].pose.position.z = 1.25;
  flag_marker_parts[1].color.r = 1.0;
  flag_marker_parts[1].color.g = 1.0;
  flag_marker_parts[1].color.b = 0.0;
  flag_marker_parts[1].color.a = 0.6;

  flag_marker_parts[2].type = visualization_msgs::msg::Marker::CYLINDER;
  flag_marker_parts[2].scale.x = 0.9;
  flag_marker_parts[2].scale.y = 0.9;
  flag_marker_parts[2].scale.z = 1.5;
  flag_marker_parts[2].pose.position.x = 0.0;
  flag_marker_parts[2].pose.position.y = 0.0;
  flag_marker_parts[2].pose.position.z = flag_marker_parts[2].scale.z * 0.5;
  flag_marker_parts[2].color.a = 0.0;

  flag_marker_parts[3].type = visualization_msgs::msg::Marker::CYLINDER;
  flag_marker_parts[3].scale.x = 2.0 * goal_reached_radius;
  flag_marker_parts[3].scale.y = 2.0 * goal_reached_radius;
  flag_marker_parts[3].scale.z = 0.03;
  flag_marker_parts[3].color.r = 0.0;
  flag_marker_parts[3].color.g = 1.0;
  flag_marker_parts[3].color.b = 0.0;
  flag_marker_parts[3].color.a = 0.3;

  if (stop_with_resume) {
    flag_marker_parts[3].color.r = 0.7;
    flag_marker_parts[3].color.g = 0.0;
    flag_marker_parts[3].color.b = 0.0;
  }

  for (const auto & marker : flag_marker_parts) {
    waypoint_control.markers.push_back(marker);
  }
}
}  // namespace nav_waypoint_visualization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav_waypoint_visualization::NavWaypointInteractiveMarkerNode)
