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

#ifndef NAV_WAYPOINT_SERVER_NODE_HPP_
#define NAV_WAYPOINT_SERVER_NODE_HPP_

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_waypoint_msgs/msg/route.hpp>
#include <nav_waypoint_msgs/srv/upsert_waypoint.hpp>
#include <nav_waypoint_msgs/srv/update_waypoint.hpp>
#include <nav_waypoint_msgs/srv/update_waypoint_pose.hpp>
#include <nav_waypoint_msgs/srv/delete_waypoint.hpp>
#include <nav_waypoint_msgs/msg/waypoint_route_configuration.hpp>
#include <nav_waypoint_msgs/srv/set_route.hpp>
#include <nav_waypoint_msgs/srv/rename_waypoint.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <nav_waypoint_server/nav_waypoint_server_node_params.hpp>

#include "waypoint_route_configuration_io.hpp"
#include "async_saver.hpp"


namespace nav_waypoint_server
{
class NavWaypointServerNode : public rclcpp::Node
{
public:
  explicit NavWaypointServerNode(const rclcpp::NodeOptions & options);

private:
  nav_waypoint_msgs::msg::WaypointRouteConfiguration configuration_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::WaypointRouteConfiguration>::SharedPtr
    configuration_publisher_;
  std::unique_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::Publisher<nav_waypoint_msgs::msg::Waypoints>::SharedPtr registered_waypoints_publisher_;
  rclcpp::Subscription<nav_waypoint_msgs::msg::Waypoint>::SharedPtr regist_, modify_;
  rclcpp::Service<nav_waypoint_msgs::srv::SetRoute>::SharedPtr set_route_service_;
  rclcpp::Service<nav_waypoint_msgs::srv::RenameWaypoint>::SharedPtr rename_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_;
  rclcpp::Service<nav_waypoint_msgs::srv::UpsertWaypoint>::SharedPtr upsert_service_;
  rclcpp::Service<nav_waypoint_msgs::srv::DeleteWaypoint>::SharedPtr delete_service_;
  rclcpp::Service<nav_waypoint_msgs::srv::UpdateWaypointPose>::SharedPtr update_pose_service_;
  rclcpp::Service<nav_waypoint_msgs::srv::UpdateWaypoint>::SharedPtr update_service_;
  AsyncSaver saver_;

  void publishConfiguration();

  void publishRegisteredWaypointsAndConfiguration();

  void validateConfiguredRoute(const nav_waypoint_msgs::msg::Route & route) const;

  void normalize(nav_waypoint_msgs::msg::Waypoint & w) const;

  void upsert(nav_waypoint_msgs::msg::Waypoint w, bool require_existing);

  nav_waypoint_msgs::msg::Waypoints loadWaypoints(const std::string & path);

  static void saveConfiguration(
    const nav_waypoint_msgs::msg::WaypointRouteConfiguration & snapshot,
    const std::string & path, const std::string & frame);
};
}  // namespace nav_waypoint_server
#endif  // NAV_WAYPOINT_SERVER_NODE_HPP_
