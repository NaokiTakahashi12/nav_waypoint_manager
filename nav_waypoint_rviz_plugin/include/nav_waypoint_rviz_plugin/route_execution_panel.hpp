// Copyright (c) 2026 Naoki Takahashi
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

#ifndef NAV_WAYPOINT_RVIZ_PLUGIN__ROUTE_EXECUTION_PANEL_HPP_
#define NAV_WAYPOINT_RVIZ_PLUGIN__ROUTE_EXECUTION_PANEL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <nav_waypoint_msgs/msg/route_progress.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

class QLabel;
class QLineEdit;
class QPushButton;

namespace nav_waypoint_rviz_plugin
{
class RouteExecutionPanel : public rviz_common::Panel
{
public:
  explicit RouteExecutionPanel(QWidget * parent = nullptr);
  ~RouteExecutionPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void initializeRos(rclcpp::Node::SharedPtr node);

private:
  using Progress = nav_waypoint_msgs::msg::RouteProgress;
  using Reset = std_srvs::srv::Empty;
  struct Inbox
  {
    std::mutex mutex;
    Progress::ConstSharedPtr latest;
  };
  std::shared_ptr<Inbox> inbox_{std::make_shared<Inbox>()};
  Progress::ConstSharedPtr progress_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Progress>::SharedPtr progress_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_publisher_;
  rclcpp::Client<Reset>::SharedPtr reset_client_;
  rclcpp::Client<Reset>::SharedFuture reset_future_;
  int64_t reset_request_id_{0};
  std::chrono::steady_clock::time_point deadline_;
  bool awaiting_progress_{false};
  std::string server_prefix_;
  QLineEdit * server_namespace_;
  QLabel * connection_;
  QLabel * state_;
  QLabel * visit_;
  QLabel * waypoint_;
  QLabel * reason_;
  QLabel * status_;
  QPushButton * resume_button_;
  QPushButton * reset_button_;

  void connectServer();
  void tick();
  void showProgress();
  void updateControls();
  void resumeRoute();
  void resetRoute();
  void cancelReset();
};
}  // namespace nav_waypoint_rviz_plugin
#endif  // NAV_WAYPOINT_RVIZ_PLUGIN__ROUTE_EXECUTION_PANEL_HPP_
