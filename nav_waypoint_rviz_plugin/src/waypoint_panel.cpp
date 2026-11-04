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

#include "nav_waypoint_rviz_plugin/waypoint_panel.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <QCheckBox>
#include <QDoubleValidator>
#include <QLocale>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <QScrollBar>
#include <cmath>
#include <algorithm>
#include <limits>
#include <nav_waypoint_msgs/srv/delete_waypoint.hpp>
#include <nav_waypoint_msgs/srv/rename_waypoint.hpp>
#include <nav_waypoint_msgs/srv/set_route.hpp>
#include <nav_waypoint_msgs/srv/update_waypoint.hpp>
#include <nav_waypoint_msgs/validation.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace nav_waypoint_rviz_plugin
{
// Persistent clients and one bounded asynchronous operation keep ROS waits off the UI thread.
template<class Service>
void WaypointPanel::request(
  std::shared_ptr<rclcpp::Client<Service>> client,
  typename Service::Request::SharedPtr message,
  std::function<void(const typename Service::Response &)> accepted)
{
  if (poll_request_) {return;}
  try {
    poll_request_ = [this, client, message, accepted,
        future = std::shared_future<typename Service::Response::SharedPtr>{}]() mutable {
        if (!future.valid()) {
          if (!client->service_is_ready()) {return false;}
          auto pending = client->async_send_request(message);
          future = pending.share();
          cancel_request_ = [client, id = pending.request_id]() {
              client->remove_pending_request(id);
            };
        }
        if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {return false;}
        const auto response = future.get();
        if (response->success && accepted) {accepted(*response);}
        status_->setText(QString::fromStdString(response->message));
        return true;
      };
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    controls_->setEnabled(false);
    status_->setText("Processing...");
  } catch (const std::exception & e) {
    poll_request_ = {}; cancel_request_ = {};
    status_->setText(QString::fromUtf8(e.what()));
    controls_->setEnabled(configuration_ != nullptr);
  }
}

template<class Service>
void WaypointPanel::mutate(
  std::shared_ptr<rclcpp::Client<Service>> client,
  typename Service::Request::SharedPtr message, AppliedEdit edit)
{
  request<Service>(
    client, message, [this, edit](const typename Service::Response & response) {
      adoptConfiguration(std::make_shared<Configuration>(response.configuration));
      if (edit == AppliedEdit::Waypoint) {form_dirty_ = false;}
      if (edit == AppliedEdit::Route) {route_dirty_ = false;}
      refresh();
    });
}

WaypointPanel::WaypointPanel(QWidget * parent)
: Panel(parent)
{
  auto layout = new QVBoxLayout(this);
  auto connection = new QHBoxLayout;
  server_namespace_ = new QLineEdit("/nav_waypoint_server", this);
  server_namespace_->setObjectName("server_namespace");
  auto connect_button = new QPushButton("Connect / Discard Edits", this);
  connection->addWidget(server_namespace_); connection->addWidget(connect_button);
  layout->addLayout(connection);
  connect(
    connect_button, &QPushButton::clicked, this, [this]() {
      connectServer(); emit configChanged();
    });
  controls_ = new QWidget(this);
  auto body = new QVBoxLayout(controls_);
  auto button = [this](QBoxLayout * row, const QString & label,
      std::function<void()> action) {
      auto b = new QPushButton(label, controls_);
      row->addWidget(b);
      connect(b, &QPushButton::clicked, this, action);
    };
  body->addWidget(new QLabel("Registered Waypoints", controls_));
  waypoints_ = new QListWidget(controls_);
  body->addWidget(waypoints_);
  auto form = new QFormLayout;
  name_ = new QLineEdit(controls_);
  name_->setReadOnly(true);
  form->addRow("Name", name_);
  const char * labels[] = {"X (m)", "Y (m)", "Z (m)",
    "Roll (rad)", "Pitch (rad)", "Yaw (rad)"};
  auto numberInput = [this](const QString & object_name) {
      auto input = new QLineEdit(controls_);
      input->setObjectName(object_name);
      auto validator = new QDoubleValidator(
        -std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(), 1000, input);
      auto locale = QLocale::c();
      locale.setNumberOptions(QLocale::RejectGroupSeparator);
      validator->setLocale(locale);
      validator->setNotation(QDoubleValidator::ScientificNotation);
      input->setValidator(validator);
      return input;
    };
  for (std::size_t i = 0; i < pose_.size(); ++i) {
    pose_[i] = numberInput("pose_" + QString::number(i));
    form->addRow(labels[i], pose_[i]);
    connect(
      pose_[i], &QLineEdit::textChanged, this, [this, i]() {
        if (!loading_) {form_dirty_ = true; pose_edited_[i] = true;}
      });
  }
  radius_ = numberInput("reach_radius");
  form->addRow("Reach radius (m)", radius_);
  stop_ = new QCheckBox("Require resume", controls_);
  form->addRow(stop_);
  body->addLayout(form);
  connect(
    radius_, &QLineEdit::textChanged, this,
    [this]() {
      if (!loading_) {
        form_dirty_ = true; radius_edited_ = true;
      }
    });
  connect(
    stop_, &QCheckBox::toggled, this, [this]() {
      if (!loading_) {
        form_dirty_ = true;
      }
    });
  auto row = new QHBoxLayout;
  body->addLayout(row);
  button(row, "Apply Waypoint", [this]() {applyWaypoint();});
  button(row, "Rename", [this]() {renameWaypoint();});
  button(row, "Delete Waypoint", [this]() {deleteWaypoint();});
  body->addWidget(
    new QLabel("Route (visit order; duplicates allowed)", controls_));
  route_ = new QListWidget(controls_);
  body->addWidget(route_);
  row = new QHBoxLayout;
  body->addLayout(row);
  button(
    row, "Append Selected", [this]() {
      if (waypoints_->currentItem()) {
        route_->addItem(waypoints_->currentItem()->text());
        route_dirty_ = true;
      }
    });
  button(row, "Up", [this]() {moveVisit(-1);});
  button(row, "Down", [this]() {moveVisit(1);});
  button(
    row, "Remove Visit", [this]() {
      if (route_->currentRow() >= 0) {
        delete route_->takeItem(route_->currentRow());
        route_dirty_ = true;
      }
    });
  row = new QHBoxLayout;
  body->addLayout(row);
  button(row, "Apply Route", [this]() {applyRoute();});
  button(
    row, "Reload / Discard Edits", [this]() {
      form_dirty_ = false;
      route_dirty_ = false;
      edited_ = {};
      refresh();
    });
  button(row, "Save Waypoints and Route", [this]() {saveConfiguration();});
  auto scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(controls_);
  layout->addWidget(scroll);
  status_ = new QLabel("Waiting for configuration", this);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  controls_->setEnabled(false);
  connect(
    waypoints_, &QListWidget::currentRowChanged, this, [this]() {
      if (form_dirty_) {
        QSignalBlocker blocker(waypoints_);
        for (int i = 0; i < waypoints_->count(); ++i) {
          if (waypoints_->item(i)->text() == name_->text()) {
            waypoints_->setCurrentRow(i);
          }
        }
        status_->setText(
          "Apply or discard waypoint edits before changing selection");
        return;
      }
      selectWaypoint();
    });
  auto timer = new QTimer(this);
  timer->setInterval(50);
  connect(timer, &QTimer::timeout, this, [this]() {tick();});
  timer->start();
}
WaypointPanel::~WaypointPanel()
{
  if (cancel_request_) {
    cancel_request_();
  }
}
void WaypointPanel::onInitialize()
{
  initializeRos(
    getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
}

void WaypointPanel::initializeRos(rclcpp::Node::SharedPtr node)
{
  node_ = std::move(node);
  connectServer();
}
void WaypointPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  QString server;
  if (config.mapGetString("Waypoint Server", &server)) {server_namespace_->setText(server);}
  if (node_) {connectServer();}
}
void WaypointPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("Waypoint Server", QString::fromStdString(server_prefix_));
}
void WaypointPanel::connectServer()
{
  if (!node_) {return;}
  if (cancel_request_) {cancel_request_();}
  poll_request_ = {}; cancel_request_ = {};  subscription_.reset();
  configuration_.reset(); inbox_ = std::make_shared<Inbox>(); retired_sources_.clear();
  form_dirty_ = false; route_dirty_ = false; edited_ = {};
  waypoints_->clear(); route_->clear(); name_->clear();
  server_prefix_ = server_namespace_->text().trimmed().toStdString();
  while (server_prefix_.size() > 1 && server_prefix_.back() == '/') {server_prefix_.pop_back();}
  controls_->setEnabled(false);
  try {
    if (server_prefix_.empty()) {throw std::invalid_argument("Specify a waypoint server name");}
    subscription_ = node_->create_subscription<Configuration>(
      server_prefix_ + "/configuration",
      rclcpp::QoS(1).transient_local(), [inbox = inbox_](Configuration::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(inbox->mutex); inbox->latest = message;
      });
    update_client_ = node_->create_client<nav_waypoint_msgs::srv::UpdateWaypoint>(
      server_prefix_ + "/update");
    rename_client_ = node_->create_client<nav_waypoint_msgs::srv::RenameWaypoint>(
      server_prefix_ + "/rename");
    delete_client_ = node_->create_client<nav_waypoint_msgs::srv::DeleteWaypoint>(
      server_prefix_ + "/delete");
    route_client_ = node_->create_client<nav_waypoint_msgs::srv::SetRoute>(
      server_prefix_ + "/set_route");
    save_client_ = node_->create_client<std_srvs::srv::Trigger>(server_prefix_ + "/save");
    status_->setText("Waiting for configuration");
  } catch (const std::exception & e) {
    subscription_.reset(); inbox_ = std::make_shared<Inbox>();
    status_->setText(QString::fromUtf8(e.what()));
  }
}

void WaypointPanel::tick()
{
  if (poll_request_) {
    try {
      if (poll_request_()) {
        poll_request_ = {}; cancel_request_ = {};
      } else if (std::chrono::steady_clock::now() >= deadline_) {
        if (cancel_request_) {cancel_request_();}
        cancel_request_ = {}; poll_request_ = {};
        status_->setText("Request timed out; result unknown. Reload before retrying.");
      }
    } catch (const std::exception & e) {
      if (cancel_request_) {cancel_request_();}
      cancel_request_ = {}; poll_request_ = {};
      status_->setText(QString::fromUtf8(e.what()));
    }
  }
  Configuration::ConstSharedPtr latest;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    latest = inbox_->latest;
  }
  if (latest) {adoptConfiguration(latest);}
  controls_->setEnabled(configuration_ && !poll_request_);
}
void WaypointPanel::adoptConfiguration(Configuration::ConstSharedPtr next)
{
  if (configuration_) {
    if (configuration_->source_id == next->source_id) {
      if (next->revision <= configuration_->revision) {return;}
    } else {
      if (retired_sources_.count(next->source_id)) {return;}
      retired_sources_.insert(configuration_->source_id);
    }
  } else {status_->setText("Ready. Use Add Waypoint to place a point.");}
  configuration_ = next;
  refresh();
}

// Modify only changed rows; preserve the selection and scroll position on pose-only updates.
static void updateItems(QListWidget * list, const std::vector<std::string> & names)
{
  QSignalBlocker blocker(list);
  const int scroll = list->verticalScrollBar()->value();
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto text = QString::fromStdString(names[i]);
    if (static_cast<int>(i) == list->count()) {
      list->addItem(text);
    } else if (list->item(i)->text() != text) {list->item(i)->setText(text);}
  }
  while (list->count() > static_cast<int>(names.size())) {delete list->takeItem(list->count() - 1);}
  list->verticalScrollBar()->setValue(scroll);
}

void WaypointPanel::refresh()
{
  if (!configuration_) {return;}
  const auto selected = name_->text();
  std::vector<std::string> names;
  for (const auto & w : configuration_->registered_waypoints.waypoints) {
    names.push_back(w.name);
  }
  updateItems(waypoints_, names);
  {
    QSignalBlocker blocker(waypoints_);
    int row = -1;
    for (int i = 0; i < waypoints_->count(); ++i) {
      if (waypoints_->item(i)->text() == selected) {row = i; break;}
    }
    if (selected.isEmpty() && waypoints_->count()) {row = waypoints_->count() - 1;}
    waypoints_->setCurrentRow(row);
  }
  if (!form_dirty_) {
    const auto found = std::find_if(
      configuration_->registered_waypoints.waypoints.begin(),
      configuration_->registered_waypoints.waypoints.end(),
      [&selected](const auto & w) {return w.name == selected.toStdString();});
    if (found == configuration_->registered_waypoints.waypoints.end() ||
      !nav_waypoint_msgs::sameWaypoint(*found, edited_)) {selectWaypoint();}
  }
  if (!route_dirty_) {
    updateItems(route_, configuration_->route.route); route_base_ = configuration_->route.route;
  }
}
void WaypointPanel::selectWaypoint()
{
  if (!configuration_) {
    return;
  }
  if (form_dirty_) {
    // Explicit selection discards this waypoint's local form, never server
    // data.
    form_dirty_ = false;
  }
  name_->clear();
  if (!waypoints_->currentItem()) {
    return;
  }
  const auto selected = waypoints_->currentItem()->text().toStdString();
  for (const auto & w : configuration_->registered_waypoints.waypoints) {
    if (w.name != selected) {
      continue;
    }
    edited_ = w;
    pose_edited_.fill(false);
    radius_edited_ = false;
    loading_ = true;
    name_->setText(QString::fromStdString(w.name));
    pose_[0]->setText(QString::number(w.pose.position.x, 'g', 17));
    pose_[1]->setText(QString::number(w.pose.position.y, 'g', 17));
    pose_[2]->setText(QString::number(w.pose.position.z, 'g', 17));
    tf2::Quaternion q;
    tf2::fromMsg(w.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_[3]->setText(QString::number(roll, 'g', 17));
    pose_[4]->setText(QString::number(pitch, 'g', 17));
    pose_[5]->setText(QString::number(yaw, 'g', 17));
    const auto settings = nav_waypoint_msgs::reachSettings(w);
    radius_->setText(QString::number(settings.first, 'g', 17));
    stop_->setChecked(settings.second);
    loading_ = false;
  }
}
void WaypointPanel::applyWaypoint()
{
  if (name_->text().isEmpty()) {
    status_->setText("Select a waypoint first");
    return;
  }
  for (const auto input : pose_) {
    bool ok = false;
    const auto value = input->text().toDouble(&ok);
    if (!input->hasAcceptableInput() || !ok || !std::isfinite(value)) {
      status_->setText("Enter finite coordinates and angles"); return;
    }
  }
  bool radius_ok = false;
  const auto radius_value = radius_->text().toDouble(&radius_ok);
  if (!radius_ok || !radius_->hasAcceptableInput() || !std::isfinite(radius_->text().toDouble()) ||
    radius_->text().toDouble() <= 0.0)
  {
    status_->setText("Reach radius must be finite and positive"); return;
  }
  const auto & points = configuration_->registered_waypoints.waypoints;
  const auto found =
    std::find_if(
    points.begin(), points.end(),
    [this](const auto & w) {return w.name == edited_.name;});
  if (found == points.end() ||
    !nav_waypoint_msgs::sameWaypoint(*found, edited_))
  {
    status_->setText("Waypoint changed externally. Reload before applying.");
    return;
  }
  auto message =
    std::make_shared<nav_waypoint_msgs::srv::UpdateWaypoint::Request>();
  message->expected_waypoint = edited_;
  message->waypoint = edited_;
  auto & w = message->waypoint;
  if (pose_edited_[0]) {w.pose.position.x = pose_[0]->text().toDouble();}
  if (pose_edited_[1]) {w.pose.position.y = pose_[1]->text().toDouble();}
  if (pose_edited_[2]) {w.pose.position.z = pose_[2]->text().toDouble();}
  if (pose_edited_[3] || pose_edited_[4] || pose_edited_[5]) {
    tf2::Quaternion q;
    tf2::fromMsg(w.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    q.setRPY(
      pose_edited_[3] ? pose_[3]->text().toDouble() : roll,
      pose_edited_[4] ? pose_[4]->text().toDouble() : pitch,
      pose_edited_[5] ? pose_[5]->text().toDouble() : yaw);
    w.pose.orientation = tf2::toMsg(q);
  }
  auto setProperty = [&w](const std::string & key, const std::string & value) {
      for (auto & p : w.properties) {
        if (p.key == key) {
          p.value = value;
          return;
        }
      }
      nav_waypoint_msgs::msg::Property p;
      p.key = key;
      p.value = value;
      w.properties.push_back(p);
    };
  if (radius_edited_) {
    setProperty(
      "goal_reached_radius",
      QString::number(radius_->text().toDouble(), 'g', 17).toStdString());
  }
  setProperty("stop_with_resume", stop_->isChecked() ? "true" : "false");
  mutate<nav_waypoint_msgs::srv::UpdateWaypoint>(update_client_, message, AppliedEdit::Waypoint);
}
void WaypointPanel::renameWaypoint()
{
  if (name_->text().isEmpty() || form_dirty_ || route_dirty_) {
    status_->setText("Apply or discard edits before renaming");
    return;
  }
  bool ok = false;
  const auto new_name =
    QInputDialog::getText(
    this, "Rename Waypoint", "New name",
    QLineEdit::Normal, name_->text(), &ok);
  if (!ok) {
    return;
  }
  auto message =
    std::make_shared<nav_waypoint_msgs::srv::RenameWaypoint::Request>();
  message->old_name = name_->text().toStdString();
  message->new_name = new_name.toStdString();
  mutate<nav_waypoint_msgs::srv::RenameWaypoint>(rename_client_, message);
}
void WaypointPanel::deleteWaypoint()
{
  if (name_->text().isEmpty() || form_dirty_ || route_dirty_) {
    status_->setText("Apply or discard edits before deleting");
    return;
  }
  auto message =
    std::make_shared<nav_waypoint_msgs::srv::DeleteWaypoint::Request>();
  message->name = name_->text().toStdString();
  mutate<nav_waypoint_msgs::srv::DeleteWaypoint>(delete_client_, message);
}
void WaypointPanel::moveVisit(int offset)
{
  const int row = route_->currentRow(), target = row + offset;
  if (row < 0 || target < 0 || target >= route_->count()) {
    return;
  }
  auto item = route_->takeItem(row);
  route_->insertItem(target, item);
  route_->setCurrentRow(target);
  route_dirty_ = true;
}
void WaypointPanel::applyRoute()
{
  if (configuration_->route.route != route_base_) {
    status_->setText("Route changed externally. Reload first.");
    return;
  }
  auto message = std::make_shared<nav_waypoint_msgs::srv::SetRoute::Request>();
  message->expected_route.header = configuration_->route.header;
  message->expected_route.route = route_base_;
  message->route.header = configuration_->route.header;
  for (int i = 0; i < route_->count(); ++i) {
    message->route.route.push_back(route_->item(i)->text().toStdString());
  }
  mutate<nav_waypoint_msgs::srv::SetRoute>(route_client_, message, AppliedEdit::Route);
}
void WaypointPanel::saveConfiguration()
{
  if (form_dirty_ || route_dirty_) {
    status_->setText("Apply or discard local edits before saving");
    return;
  }
  request<std_srvs::srv::Trigger>(
    save_client_, std::make_shared<std_srvs::srv::Trigger::Request>());
}
}  // namespace nav_waypoint_rviz_plugin
PLUGINLIB_EXPORT_CLASS(
  nav_waypoint_rviz_plugin::WaypointPanel,
  rviz_common::Panel)
