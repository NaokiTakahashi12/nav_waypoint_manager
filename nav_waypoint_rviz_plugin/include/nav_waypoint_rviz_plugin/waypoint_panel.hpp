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

#ifndef NAV_WAYPOINT_RVIZ_PLUGIN__WAYPOINT_PANEL_HPP_
#define NAV_WAYPOINT_RVIZ_PLUGIN__WAYPOINT_PANEL_HPP_
#include <QWidget>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <vector>
#include <string>
#include <nav_waypoint_msgs/msg/waypoint_route_configuration.hpp>
#include <nav_waypoint_msgs/srv/update_waypoint.hpp>
#include <nav_waypoint_msgs/srv/rename_waypoint.hpp>
#include <nav_waypoint_msgs/srv/delete_waypoint.hpp>
#include <nav_waypoint_msgs/srv/set_route.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
class QListWidget;
class QLineEdit;
class QCheckBox;
class QLabel;
namespace nav_waypoint_rviz_plugin
{
class WaypointPanel : public rviz_common::Panel
{
public:
  explicit WaypointPanel(QWidget * parent = nullptr);
  ~WaypointPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void initializeRos(rclcpp::Node::SharedPtr node);

private:
  using Configuration = nav_waypoint_msgs::msg::WaypointRouteConfiguration;
  struct Inbox
  {
    std::mutex mutex;
    Configuration::ConstSharedPtr latest;
  };
  std::shared_ptr<Inbox> inbox_{std::make_shared<Inbox>()};
  Configuration::ConstSharedPtr configuration_;
  std::set<std::string> retired_sources_;
  rclcpp::Client<nav_waypoint_msgs::srv::UpdateWaypoint>::SharedPtr update_client_;
  rclcpp::Client<nav_waypoint_msgs::srv::RenameWaypoint>::SharedPtr rename_client_;
  rclcpp::Client<nav_waypoint_msgs::srv::DeleteWaypoint>::SharedPtr delete_client_;
  rclcpp::Client<nav_waypoint_msgs::srv::SetRoute>::SharedPtr route_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Configuration>::SharedPtr subscription_;
  QLineEdit * server_namespace_;
  std::string server_prefix_;
  void connectServer();
  QWidget * controls_;
  QListWidget * waypoints_;
  QListWidget * route_;
  QLineEdit * name_;
  std::array<QLineEdit *, 6> pose_;
  QLineEdit * radius_;
  QCheckBox * stop_;
  QLabel * status_;
  nav_waypoint_msgs::msg::Waypoint edited_;
  bool form_dirty_{false};
  std::array<bool, 6> pose_edited_{};
  bool radius_edited_{false};
  bool route_dirty_{false};
  bool loading_{false};
  std::vector<std::string> route_base_;
  std::function<bool()> poll_request_;
  std::function<void()> cancel_request_;
  std::chrono::steady_clock::time_point deadline_;
  enum class AppliedEdit {None, Waypoint, Route};
  enum class ConfigurationResult {RetiredSource, Unchanged, Updated};
  ConfigurationResult adoptConfiguration(Configuration::ConstSharedPtr configuration);
  void tick();
  void refresh();
  void selectWaypoint();
  void applyWaypoint();
  void renameWaypoint();
  void deleteWaypoint();
  void applyRoute();
  void moveVisit(int offset);
  void saveConfiguration();
  template<class Service>
  void request(
    std::shared_ptr<rclcpp::Client<Service>> client,
    typename Service::Request::SharedPtr message,
    std::function<bool(const typename Service::Response &)> accepted = {});
  template<class Service>
  void mutate(
    std::shared_ptr<rclcpp::Client<Service>> client, typename Service::Request::SharedPtr message,
    AppliedEdit edit = AppliedEdit::None);
};
}  // namespace nav_waypoint_rviz_plugin
#endif  // NAV_WAYPOINT_RVIZ_PLUGIN__WAYPOINT_PANEL_HPP_
