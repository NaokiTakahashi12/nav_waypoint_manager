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

#include <QApplication>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QThread>
#include <gtest/gtest.h>
#include <functional>
#include <pluginlib/class_loader.hpp>
#include "nav_waypoint_rviz_plugin/route_execution_panel.hpp"

TEST(RouteExecutionWorkflow, ProgressCommandsResetAndReconnect) {
  int argc = 0;
  QApplication app(argc, nullptr);
  rclcpp::init(argc, nullptr);
  auto node = std::make_shared<rclcpp::Node>("route_execution_panel_test");
  {
    using Progress = nav_waypoint_msgs::msg::RouteProgress;
    using Reset = std_srvs::srv::Empty;
    auto publisher = node->create_publisher<Progress>(
      "/robot1/nav_route_server/progress", rclcpp::QoS(1).transient_local());
    int resumes = 0, resets = 0;
    auto subscriber = node->create_subscription<std_msgs::msg::Bool>(
      "/robot1/nav_route_server/release_resume", 10,
      [&](std_msgs::msg::Bool::ConstSharedPtr message) {
        EXPECT_TRUE(message->data); ++resumes;
      });
    rclcpp::Service<Reset>::SharedPtr pending_service;
    std::shared_ptr<rmw_request_id_t> pending_header;
    auto service = node->create_service<Reset>(
      "/robot1/nav_route_server/reset",
      [&](rclcpp::Service<Reset>::SharedPtr current, std::shared_ptr<rmw_request_id_t> header,
      Reset::Request::SharedPtr) {
        pending_service = current; pending_header = header; ++resets;
      });
    pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common", "rviz_common::Panel");
    auto instance = panels.createSharedInstance("nav_waypoint_rviz_plugin/RouteExecutionPanel");
    auto panel = dynamic_cast<nav_waypoint_rviz_plugin::RouteExecutionPanel *>(instance.get());
    ASSERT_NE(panel, nullptr);
    rviz_common::Config config;
    config.mapSetValue("Route Server", "/robot1/nav_route_server/");
    panel->load(config);
    panel->initializeRos(node);
    auto start = panel->findChild<QPushButton *>("route_resume");
    auto reset = panel->findChild<QPushButton *>("route_reset");
    auto state = panel->findChild<QLabel *>("route_state");
    auto visit = panel->findChild<QLabel *>("route_visit");
    auto waypoint = panel->findChild<QLabel *>("route_waypoint");
    auto reason = panel->findChild<QLabel *>("route_reason");
    auto status = panel->findChild<QLabel *>("route_operation_status");
    auto server = panel->findChild<QLineEdit *>("route_server_namespace");
    ASSERT_NE(start, nullptr); ASSERT_NE(reset, nullptr); ASSERT_NE(state, nullptr);
    ASSERT_NE(visit, nullptr); ASSERT_NE(waypoint, nullptr); ASSERT_NE(reason, nullptr);
    ASSERT_NE(status, nullptr); ASSERT_NE(server, nullptr);
    auto wait = [&](std::function<bool()> condition, int milliseconds = 5000) {
        const auto end = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(milliseconds);
        while (std::chrono::steady_clock::now() < end) {
          rclcpp::spin_some(node); app.processEvents(); QThread::msleep(10);
          if (condition()) {return true;}
        }
        return false;
      };
    auto spinFor = [&](int milliseconds) {wait([]() {return false;}, milliseconds);};
    auto reconnect = [&]() {
        for (auto button : panel->findChildren<QPushButton *>()) {
          if (button->text() == "Connect") {button->click(); return true;}
        }
        return false;
      };
    EXPECT_FALSE(start->isEnabled()); EXPECT_FALSE(reset->isEnabled());
    ASSERT_TRUE(wait([&]() {return publisher->get_subscription_count() > 0;}));
    EXPECT_FALSE(start->isEnabled()); EXPECT_FALSE(reset->isEnabled());
    Progress progress;
    progress.state = "startup_wait"; progress.route_index = 0;
    progress.route_size = 3; progress.waypoint_name = "alpha";
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return start->isEnabled() && reset->isEnabled();}));
    EXPECT_EQ(state->text(), "Awaiting start");
    EXPECT_EQ(visit->text(), "1 / 3"); EXPECT_EQ(waypoint->text(), "alpha");
    EXPECT_EQ(start->text(), "Start Route");
    rviz_common::Config saved;
    panel->save(saved);
    QString saved_server;
    ASSERT_TRUE(saved.mapGetString("Route Server", &saved_server));
    EXPECT_EQ(saved_server, "/robot1/nav_route_server");

    start->click(); start->click();  // A pending command cannot be issued twice.
    ASSERT_TRUE(wait([&]() {return resumes == 1;}));
    spinFor(350);
    EXPECT_EQ(resumes, 1); EXPECT_FALSE(start->isEnabled());
    progress.state = "running"; publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return state->text() == "Following route";}));
    EXPECT_FALSE(status->text().contains("Awaiting route progress"));
    EXPECT_FALSE(start->isEnabled());
    progress.state = "waiting_resume"; progress.route_index = 1;
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return start->isEnabled();}));
    EXPECT_EQ(start->text(), "Resume Route");
    EXPECT_EQ(visit->text(), "2 / 3"); EXPECT_EQ(waypoint->text(), "alpha");
    spinFor(350);  // Progress is change-only; quiet topics do not invalidate it.
    EXPECT_TRUE(start->isEnabled()); EXPECT_EQ(resumes, 1);
    start->click();
    ASSERT_TRUE(wait([&]() {return resumes == 2;}));
    progress.state = "running"; progress.route_index = 2; progress.waypoint_name = "beta";
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return waypoint->text() == "beta";}));
    EXPECT_EQ(visit->text(), "3 / 3"); EXPECT_FALSE(start->isEnabled());

    reset->click(); reset->click();
    ASSERT_TRUE(wait([&]() {return resets == 1;}));
    EXPECT_FALSE(start->isEnabled()); EXPECT_FALSE(reset->isEnabled());
    progress.state = "startup_wait"; progress.route_index = 0; progress.waypoint_name = "alpha";
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return state->text() == "Awaiting start";}));
    EXPECT_FALSE(start->isEnabled());  // The reset response is still pending.
    Reset::Response response;
    pending_service->send_response(*pending_header, response);
    ASSERT_TRUE(wait([&]() {return start->isEnabled() && reset->isEnabled();}));
    EXPECT_EQ(resumes, 2);  // Reset does not automatically start the route.
    reset->click();
    ASSERT_TRUE(wait([&]() {return resets == 2;}));
    pending_service->send_response(*pending_header, response);
    // No new progress is required for resetting an already waiting first visit.
    ASSERT_TRUE(wait([&]() {return start->isEnabled();}));

    progress.state = "completed"; progress.route_index = 3; progress.waypoint_name.clear();
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return state->text() == "Route completed";}));
    EXPECT_EQ(visit->text(), "3 / 3"); EXPECT_EQ(waypoint->text(), "-");
    EXPECT_FALSE(start->isEnabled());
    progress.state = "empty"; progress.route_index = 0; progress.route_size = 0;
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return state->text() == "No route";}));
    EXPECT_EQ(visit->text(), "0 / 0"); EXPECT_FALSE(start->isEnabled());
    progress.state = "disabled"; progress.route_index = -1; progress.route_size = 3;
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return state->text() == "Route disabled";}));
    EXPECT_EQ(visit->text(), "- / 3"); EXPECT_FALSE(start->isEnabled());
    progress.state = "blocked"; progress.route_index = 1;
    progress.reason = "Robot transform unavailable";
    publisher->publish(progress);
    ASSERT_TRUE(wait([&]() {return reason->text() == "Robot transform unavailable";}));
    EXPECT_EQ(state->text(), "Progress blocked"); EXPECT_FALSE(start->isEnabled());

    // An unresponsive reset is bounded and never generates automatic retries or start commands.
    reset->click();
    ASSERT_TRUE(wait([&]() {return resets == 3;}));
    ASSERT_TRUE(wait([&]() {return status->text().contains("timed out");}, 6500));
    EXPECT_EQ(resets, 3); EXPECT_EQ(resumes, 2);
    EXPECT_TRUE(reset->isEnabled()); EXPECT_FALSE(start->isEnabled());

    // Reconnecting cancels pending work and drops callbacks belonging to the previous connection.
    reset->click();
    ASSERT_TRUE(wait([&]() {return resets == 4;}));
    server->setText("bad namespace"); ASSERT_TRUE(reconnect());
    EXPECT_FALSE(start->isEnabled()); EXPECT_FALSE(reset->isEnabled());
    pending_service->send_response(*pending_header, response);
    spinFor(250);
    EXPECT_FALSE(start->isEnabled()); EXPECT_FALSE(reset->isEnabled());
    EXPECT_FALSE(status->text().contains("acknowledged"));
    server->setText("/other/nav_route_server"); ASSERT_TRUE(reconnect());
    progress.state = "startup_wait"; progress.route_index = 0; progress.reason.clear();
    progress.waypoint_name = "alpha"; publisher->publish(progress);
    spinFor(250);
    EXPECT_EQ(state->text(), "-"); EXPECT_FALSE(start->isEnabled());
    server->setText("/robot1/nav_route_server"); ASSERT_TRUE(reconnect());
    // Latched progress works after reconnect.
    ASSERT_TRUE(wait([&]() {return start->isEnabled();}));
    EXPECT_EQ(visit->text(), "1 / 3");

    // Missing command endpoints disable the relevant buttons without changing route state.
    subscriber.reset();
    ASSERT_TRUE(wait([&]() {return !start->isEnabled();}));
    pending_service.reset(); service.reset();
    ASSERT_TRUE(wait([&]() {return !reset->isEnabled();}));
    publisher.reset();
    spinFor(250);
    EXPECT_FALSE(start->isEnabled()); EXPECT_FALSE(reset->isEnabled());
    EXPECT_EQ(resumes, 2);
  }
  node.reset(); rclcpp::shutdown();
}
