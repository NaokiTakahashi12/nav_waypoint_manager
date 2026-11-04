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

#include <QApplication>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QThread>
#include <QLabel>
#include <gtest/gtest.h>
#include "nav_waypoint_rviz_plugin/waypoint_panel.hpp"
#include "nav_waypoint_rviz_plugin/add_waypoint_tool.hpp"
#include <nav_waypoint_msgs/srv/set_route.hpp>
#include <nav_waypoint_msgs/srv/delete_waypoint.hpp>
#include <nav_waypoint_msgs/srv/update_waypoint.hpp>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/tool.hpp>
#include <std_srvs/srv/trigger.hpp>
class PlacementProbe : public nav_waypoint_rviz_plugin::AddWaypointTool
{
public:
  using AddWaypointTool::publishPose;
};
TEST(PanelWorkflow, PluginLoadingEditsRouteAndSave) {
  int argc = 0;
  QApplication app(argc, nullptr);
  rclcpp::init(argc, nullptr);
  auto node = std::make_shared<rclcpp::Node>("panel_test");
  {
    pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common",
      "rviz_common::Panel");
    pluginlib::ClassLoader<rviz_common::Tool> tools("rviz_common",
      "rviz_common::Tool");
    auto tool =
      tools.createSharedInstance("nav_waypoint_rviz_plugin/AddWaypoint");
    ASSERT_NE(tool, nullptr);
    auto instance =
      panels.createSharedInstance("nav_waypoint_rviz_plugin/WaypointPanel");
    auto panel =
      dynamic_cast<nav_waypoint_rviz_plugin::WaypointPanel *>(instance.get());
    ASSERT_NE(panel, nullptr);
    panel->initializeRos(node);
    using Configuration = nav_waypoint_msgs::msg::WaypointRouteConfiguration;
    Configuration configuration;
    configuration.source_id = "test_server";
    configuration.revision = 1;
    configuration.registered_waypoints.header.frame_id =
      configuration.route.header.frame_id = "map";
    for (const auto & name : {"alpha", "beta"}) {
      nav_waypoint_msgs::msg::Waypoint waypoint;
      waypoint.name = name;
      waypoint.frame_id = "map";
      waypoint.pose.orientation.w = 1.0;
      nav_waypoint_msgs::msg::Property property;
      property.key = "custom";
      property.value = "keep";
      waypoint.properties.push_back(property);
      configuration.registered_waypoints.waypoints.push_back(waypoint);
    }
    configuration.route.route = {"alpha", "alpha"};
    auto publisher = node->create_publisher<Configuration>(
      "nav_waypoint_server/configuration", rclcpp::QoS(1).transient_local());
    int edited = 0, routes = 0, saved = 0;
    auto upsert = node->create_service<nav_waypoint_msgs::srv::UpdateWaypoint>(
      "nav_waypoint_server/update",
      [&](nav_waypoint_msgs::srv::UpdateWaypoint::Request::SharedPtr request,
      nav_waypoint_msgs::srv::UpdateWaypoint::Response::SharedPtr
      response) {
        EXPECT_EQ(request->expected_waypoint, configuration.registered_waypoints.waypoints[0]);
        configuration.registered_waypoints.waypoints[0] = request->waypoint;
        ++configuration.revision;
        response->configuration = configuration;
        response->success = true;
        response->message = "updated";
        ++edited;
        // Do not publish here: the response alone must update the panel.
      });
    auto set_route = node->create_service<nav_waypoint_msgs::srv::SetRoute>(
      "nav_waypoint_server/set_route",
      [&](nav_waypoint_msgs::srv::SetRoute::Request::SharedPtr request,
      nav_waypoint_msgs::srv::SetRoute::Response::SharedPtr response) {
        EXPECT_EQ(request->expected_route.route, configuration.route.route);
        configuration.route = request->route;
        ++configuration.revision;
        response->configuration = configuration;
        response->success = true;
        ++routes;
        publisher->publish(configuration);
      });
    auto reject_delete = node->create_service<nav_waypoint_msgs::srv::DeleteWaypoint>(
      "nav_waypoint_server/delete",
      [](nav_waypoint_msgs::srv::DeleteWaypoint::Request::SharedPtr,
      nav_waypoint_msgs::srv::DeleteWaypoint::Response::SharedPtr response) {
        response->success = false; response->message = "Referenced waypoint cannot be deleted";
      });
    auto save = node->create_service<std_srvs::srv::Trigger>(
      "nav_waypoint_server/save",
      [&](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = true;
        ++saved;
      });
    auto wait = [&](std::function<bool()> condition) {
        const auto end =
          std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < end) {
          rclcpp::spin_some(node);
          app.processEvents();
          QThread::msleep(10);
          if (condition()) {
            return true;
          }
        }
        return false;
      };
    auto idle = [&]() {
        for (auto button : panel->findChildren<QPushButton *>()) {
          if (button->text() == "Apply Waypoint") {return button->isEnabled();}
        }
        return false;
      };
    auto click = [&](const QString & text) {
        for (auto button : panel->findChildren<QPushButton *>()) {
          if (button->text() == text && button->isEnabled()) {
            button->click();
            return true;
          }
        }
        return false;
      };
    PlacementProbe placement;
    placement.initializeRos(node);
    int registrations = 0, navigation_goals = 0;
    auto registration = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/set_waypoint", 10, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {
        EXPECT_EQ(pose->header.frame_id, "map"); EXPECT_DOUBLE_EQ(pose->pose.position.x, 2.0);
        ++registrations;
      });
    auto navigation = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {++navigation_goals;});
    ASSERT_TRUE(wait([&]() {placement.publishPose(2, 3, 0.5, "map"); return registrations > 0;}));
    EXPECT_EQ(navigation_goals, 0);
    publisher->publish(configuration);
    auto lists = panel->findChildren<QListWidget *>();
    ASSERT_EQ(lists.size(), 2);
    ASSERT_TRUE(wait([&]() {return lists[0]->count() == 2;}));
    lists[0]->setCurrentRow(0);
    ASSERT_TRUE(click("Delete Waypoint"));
    ASSERT_TRUE(
      wait(
        [&]() {
          for (auto label : panel->findChildren<QLabel *>()) {
            if (label->text().contains("cannot be deleted")) {return true;}
          }
          return false;
        }));
    std::vector<QLineEdit *> values;
    for (int i = 0; i < 6; ++i) {
      values.push_back(panel->findChild<QLineEdit *>("pose_" + QString::number(i)));
    }
    values.push_back(panel->findChild<QLineEdit *>("reach_radius"));
    for (auto value : values) {
      ASSERT_NE(value, nullptr);
    }
    values[0]->setText("6");
    ASSERT_TRUE(click("Append Selected"));
    ASSERT_TRUE(click("Apply Route"));
    ASSERT_TRUE(wait([&]() {return routes == 1 && idle();}));
    EXPECT_EQ(values[0]->text(), "6"); // applying route must preserve the waypoint draft
    ASSERT_TRUE(click("Append Selected"));
    ASSERT_TRUE(click("Apply Waypoint"));
    ASSERT_TRUE(
      wait(
        [&]() {
          return edited == 1 &&
          idle();
        }));
    EXPECT_EQ(
      configuration.registered_waypoints.waypoints[0].pose.position.x,
      6.0);
    EXPECT_EQ(
      configuration.registered_waypoints.waypoints[0].properties[0].value,
      "keep");
    EXPECT_EQ(lists[1]->count(), 4); // waypoint apply must not discard route draft
    auto stale = configuration; stale.revision = 1;
    stale.registered_waypoints.waypoints[0].pose.position.x = 0;
    publisher->publish(stale);
    ASSERT_TRUE(wait([&]() {return values[0]->text().toDouble() == 6.0;}));
    lists[1]->setCurrentRow(2); ASSERT_TRUE(click("Remove Visit"));
    const auto saved_pose = configuration.registered_waypoints.waypoints[0].pose;
    values[6]->setText("0");
    ASSERT_TRUE(click("Apply Waypoint")); EXPECT_EQ(edited, 1);
    values[6]->setText("7e-40");
    ASSERT_TRUE(click("Apply Waypoint"));
    ASSERT_TRUE(
      wait(
        [&]() {
          return edited == 2 && idle();
        }));
    EXPECT_EQ(configuration.registered_waypoints.waypoints[0].pose, saved_pose);
    lists[0]->setCurrentRow(1);
    ASSERT_TRUE(click("Append Selected"));
    lists[1]->setCurrentRow(lists[1]->count() - 1);
    ASSERT_TRUE(click("Up"));
    ASSERT_TRUE(click("Save Waypoints and Route"));
    EXPECT_EQ(saved, 0);
    ASSERT_TRUE(click("Apply Route"));
    ASSERT_TRUE(
      wait(
        [&]() {
          return routes == 2 &&
          idle();
        }));
    EXPECT_EQ(
      configuration.route.route,
      (std::vector<std::string>{"alpha", "alpha", "beta", "alpha"}));
    ASSERT_TRUE(click("Save Waypoints and Route"));
    ASSERT_TRUE(wait([&]() {return saved == 1 && idle();}));
    auto server_name = panel->findChild<QLineEdit *>("server_namespace");
    server_name->setText("bad namespace"); ASSERT_TRUE(click("Connect / Discard Edits"));
    EXPECT_FALSE(idle());
    server_name->setText("/nav_waypoint_server");
    ASSERT_TRUE(click("Connect / Discard Edits"));
    ASSERT_TRUE(wait([&]() {return idle();}));
  }
  node.reset();
  rclcpp::shutdown();
}
