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
#include <QInputDialog>
#include <QTimer>
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
    EXPECT_EQ(values[0]->text(), "6");  // applying route must preserve the waypoint draft
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
    EXPECT_EQ(lists[1]->count(), 4);  // waypoint apply must not discard route draft
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

    // Defer replies so configuration delivery can be ordered independently of RPC completion.
    using Update = nav_waypoint_msgs::srv::UpdateWaypoint;
    rclcpp::Service<Update>::SharedPtr pending_service;
    std::shared_ptr<rmw_request_id_t> pending_header;
    Update::Request::SharedPtr pending_update;
    upsert.reset();
    upsert = node->create_service<Update>(
      "nav_waypoint_server/update",
      [&](rclcpp::Service<Update>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
      Update::Request::SharedPtr message) {
        pending_service = service; pending_header = header; pending_update = message;
      });
    auto addPoint = [&](const std::string & name) {
        auto point = configuration.registered_waypoints.waypoints.back();
        point.name = name; configuration.registered_waypoints.waypoints.push_back(point);
      };
    auto hasStatus = [&](const QString & text) {
        for (auto label : panel->findChildren<QLabel *>()) {
          if (label->text().contains(text)) {return true;}
        }
        return false;
      };
    lists[0]->setCurrentRow(0);
    values[0]->setText("7");
    ASSERT_TRUE(click("Apply Waypoint"));
    ASSERT_TRUE(wait([&]() {return pending_update != nullptr;}));
    Update::Response response;
    response.success = true; response.message = "updated";
    response.configuration = configuration;
    ++response.configuration.revision;
    response.configuration.registered_waypoints.waypoints[0] = pending_update->waypoint;
    configuration.revision += 2;
    addPoint("newer_snapshot"); publisher->publish(configuration);
    ASSERT_TRUE(wait([&]() {return lists[0]->count() == 3;}));
    pending_service->send_response(*pending_header, response);
    ASSERT_TRUE(wait([&]() {return idle();}));
    // A valid response older than the current snapshot completes the edit using current values.
    EXPECT_DOUBLE_EQ(values[0]->text().toDouble(), 6.0);
    ASSERT_TRUE(click("Save Waypoints and Route"));
    ASSERT_TRUE(wait([&]() {return saved == 2 && idle();}));

    pending_update.reset();
    values[0]->setText("9");
    ASSERT_TRUE(click("Append Selected"));
    ASSERT_TRUE(click("Apply Waypoint"));
    ASSERT_TRUE(wait([&]() {return pending_update != nullptr;}));
    response.configuration = configuration;
    ++response.configuration.revision;
    response.configuration.registered_waypoints.waypoints[0] = pending_update->waypoint;
    response.message = "previous server accepted";
    configuration.source_id = "restarted_server"; configuration.revision = 1;
    addPoint("after_restart"); publisher->publish(configuration);
    ASSERT_TRUE(wait([&]() {return lists[0]->count() == 4;}));
    pending_service->send_response(*pending_header, response);
    ASSERT_TRUE(wait([&]() {return idle();}));
    EXPECT_TRUE(hasStatus("previous server response ignored"));
    EXPECT_DOUBLE_EQ(values[0]->text().toDouble(), 9.0);
    ASSERT_TRUE(click("Apply Route"));
    ASSERT_TRUE(wait([&]() {return routes == 3 && idle();}));
    ASSERT_TRUE(click("Save Waypoints and Route"));
    EXPECT_TRUE(hasStatus("Apply or discard local edits"));
    EXPECT_EQ(saved, 2);  // The old waypoint response must leave the form draft dirty.

    pending_update.reset();
    ASSERT_TRUE(click("Apply Waypoint"));
    ASSERT_TRUE(wait([&]() {return pending_update != nullptr;}));
    EXPECT_DOUBLE_EQ(pending_update->expected_waypoint.pose.position.x, 6.0);
    EXPECT_DOUBLE_EQ(pending_update->waypoint.pose.position.x, 9.0);
    configuration.registered_waypoints.waypoints[0] = pending_update->waypoint;
    ++configuration.revision; response.configuration = configuration;
    pending_service->send_response(*pending_header, response);
    ASSERT_TRUE(wait([&]() {return idle();}));
    ASSERT_TRUE(click("Save Waypoints and Route"));
    ASSERT_TRUE(wait([&]() {return saved == 3 && idle();}));

    using SetRoute = nav_waypoint_msgs::srv::SetRoute;
    rclcpp::Service<SetRoute>::SharedPtr pending_route_service;
    std::shared_ptr<rmw_request_id_t> pending_route_header;
    SetRoute::Request::SharedPtr pending_route;
    set_route.reset();
    set_route = node->create_service<SetRoute>(
      "nav_waypoint_server/set_route",
      [&](rclcpp::Service<SetRoute>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
      SetRoute::Request::SharedPtr message) {
        pending_route_service = service; pending_route_header = header; pending_route = message;
      });
    ASSERT_TRUE(click("Append Selected"));
    ASSERT_TRUE(click("Apply Route"));
    ASSERT_TRUE(wait([&]() {return pending_route != nullptr;}));
    SetRoute::Response route_response;
    route_response.success = true; route_response.message = "previous route accepted";
    route_response.configuration = configuration;
    ++route_response.configuration.revision;
    route_response.configuration.route = pending_route->route;
    configuration.source_id = "next_server"; configuration.revision = 1;
    addPoint("next_restart"); publisher->publish(configuration);
    ASSERT_TRUE(wait([&]() {return lists[0]->count() == 5;}));
    pending_route_service->send_response(*pending_route_header, route_response);
    ASSERT_TRUE(wait([&]() {return idle();}));
    EXPECT_TRUE(hasStatus("previous server response ignored"));
    EXPECT_EQ(lists[1]->count(), static_cast<int>(pending_route->route.route.size()));
    ASSERT_TRUE(click("Save Waypoints and Route"));
    EXPECT_TRUE(hasStatus("Apply or discard local edits"));
    EXPECT_EQ(saved, 3);
    ASSERT_TRUE(click("Reload / Discard Edits"));

    std::string rename_target;
    auto rename = node->create_service<nav_waypoint_msgs::srv::RenameWaypoint>(
      "nav_waypoint_server/rename",
      [&](nav_waypoint_msgs::srv::RenameWaypoint::Request::SharedPtr message,
      nav_waypoint_msgs::srv::RenameWaypoint::Response::SharedPtr result) {
        rename_target = message->old_name;
        result->success = false; result->message = "Original target no longer exists";
      });
    lists[0]->setCurrentRow(0);
    // ROS delivery must continue inside QInputDialog's nested event loop.
    QTimer dialog_updates;
    int dialog_step = 0;
    QObject::connect(
      &dialog_updates, &QTimer::timeout, [&]() {
        rclcpp::spin_some(node);
        auto dialog = qobject_cast<QInputDialog *>(QApplication::activeModalWidget());
        if (!dialog) {return;}
        if (dialog_step == 0) {
          configuration.registered_waypoints.waypoints.erase(
            configuration.registered_waypoints.waypoints.begin());
          configuration.route.route.clear();
          ++configuration.revision; publisher->publish(configuration); ++dialog_step;
        } else if (dialog_step == 1 && lists[0]->count() == 4) {
          ++configuration.revision; publisher->publish(configuration); ++dialog_step;
        } else if (dialog_step == 2 && lists[0]->currentRow() >= 0) {
          dialog->setTextValue("renamed"); dialog->accept();
        }
      });
    dialog_updates.start(10);
    QTimer::singleShot(
      5000, &dialog_updates, [&]() {
        if (auto dialog = qobject_cast<QInputDialog *>(QApplication::activeModalWidget())) {
          dialog->reject();
        }
      });
    ASSERT_TRUE(click("Rename"));
    dialog_updates.stop();
    ASSERT_TRUE(wait([&]() {return !rename_target.empty() && idle();}));
    EXPECT_EQ(rename_target, "alpha");
  }
  node.reset();
  rclcpp::shutdown();
}
