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

#include "nav_waypoint_rviz_plugin/route_execution_panel.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <future>
#include <stdexcept>
#include <utility>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include "server_candidates.hpp"

namespace nav_waypoint_rviz_plugin
{
RouteExecutionPanel::RouteExecutionPanel(QWidget * parent)
: Panel(parent)
{
  auto layout = new QVBoxLayout(this);
  auto connection = new QHBoxLayout;
  server_namespace_ = new QComboBox(this);
  server_namespace_->setObjectName("route_server_candidates");
  server_namespace_->setEditable(true);
  server_namespace_->setInsertPolicy(QComboBox::NoInsert);
  server_namespace_->setToolTip("Select or enter a route server, then click Connect");
  server_namespace_->setEditText("/nav_route_server");
  server_namespace_->lineEdit()->setObjectName("route_server_namespace");
  auto refresh_button = new QPushButton("Refresh", this);
  refresh_button->setObjectName("refresh_route_servers");
  refresh_button->setToolTip("Refresh servers publishing route progress");
  auto connect_button = new QPushButton("Connect", this);
  connection->addWidget(server_namespace_, 1);
  connection->addWidget(refresh_button); connection->addWidget(connect_button);
  layout->addLayout(connection);
  connect(refresh_button, &QPushButton::clicked, this, [this]() {refreshServers();});
  connect(
    connect_button, &QPushButton::clicked, this, [this]() {
      connectServer(); emit configChanged();
    });
  connection_ = new QLabel("Waiting for route progress", this);
  connection_->setObjectName("route_connection");
  connection_->setWordWrap(true);
  layout->addWidget(connection_);
  auto form = new QFormLayout;
  auto field = [this, form](const QString & title, const QString & object_name) {
      auto label = new QLabel("-", this);
      label->setObjectName(object_name);
      label->setWordWrap(true);
      label->setTextFormat(Qt::PlainText);
      form->addRow(title, label);
      return label;
    };
  state_ = field("Route state", "route_state");
  visit_ = field("Visit", "route_visit");
  waypoint_ = field("Waypoint", "route_waypoint");
  reason_ = field("Reason", "route_reason");
  layout->addLayout(form);
  resume_button_ = new QPushButton("Start Route", this);
  resume_button_->setObjectName("route_resume");
  reset_button_ = new QPushButton("Reset to First Visit", this);
  reset_button_->setObjectName("route_reset");
  reset_button_->setToolTip(
    "Return to the first visit and await Start Route. This does not stop the robot.");
  layout->addWidget(resume_button_); layout->addWidget(reset_button_);
  connect(resume_button_, &QPushButton::clicked, this, [this]() {resumeRoute();});
  connect(reset_button_, &QPushButton::clicked, this, [this]() {resetRoute();});
  status_ = new QLabel(this);
  status_->setObjectName("route_operation_status");
  status_->setTextFormat(Qt::PlainText);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  layout->addStretch();
  updateControls();
  auto timer = new QTimer(this);
  timer->setInterval(100);
  connect(timer, &QTimer::timeout, this, [this]() {tick();});
  timer->start();
}

RouteExecutionPanel::~RouteExecutionPanel()
{
  cancelReset();
}

void RouteExecutionPanel::onInitialize()
{
  initializeRos(getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
}

void RouteExecutionPanel::initializeRos(rclcpp::Node::SharedPtr node)
{
  node_ = std::move(node);
  refreshServers();
  connectServer();
}

void RouteExecutionPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  QString server;
  if (config.mapGetString("Route Server", &server)) {server_namespace_->setEditText(server);}
  if (node_) {connectServer();}
}

void RouteExecutionPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("Route Server", QString::fromStdString(server_prefix_));
}

void RouteExecutionPanel::connectServer()
{
  if (!node_) {return;}
  cancelReset();
  progress_subscription_.reset(); resume_publisher_.reset(); reset_client_.reset();
  inbox_ = std::make_shared<Inbox>(); progress_.reset(); awaiting_progress_ = false;
  server_prefix_ = server_namespace_->currentText().trimmed().toStdString();
  while (server_prefix_.size() > 1 && server_prefix_.back() == '/') {server_prefix_.pop_back();}
  status_->clear(); showProgress();
  try {
    if (server_prefix_.empty()) {throw std::invalid_argument("Specify a route server name");}
    resume_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
      server_prefix_ + "/release_resume", 1);
    reset_client_ = node_->create_client<Reset>(server_prefix_ + "/reset");
    progress_subscription_ = node_->create_subscription<Progress>(
      server_prefix_ + "/progress", rclcpp::QoS(1).transient_local(),
      [inbox = inbox_](Progress::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(inbox->mutex); inbox->latest = message;
      });
  } catch (const std::exception & e) {
    progress_subscription_.reset(); resume_publisher_.reset(); reset_client_.reset();
    status_->setText(QString::fromUtf8(e.what()));
  }
  updateControls();
}

void RouteExecutionPanel::refreshServers()
{
  if (!node_) {return;}
  try {
    refreshServerCandidates(
      *server_namespace_, *node_, "/progress", "nav_waypoint_msgs/msg/RouteProgress");
  } catch (const std::exception & e) {
    status_->setText(QString::fromUtf8(e.what()));
  }
}

void RouteExecutionPanel::tick()
{
  Progress::ConstSharedPtr latest;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    latest = std::move(inbox_->latest);
  }
  if (latest) {
    if (awaiting_progress_) {status_->clear();}
    progress_ = std::move(latest); awaiting_progress_ = false;
    showProgress();
  }
  if (reset_future_.valid()) {
    try {
      if (reset_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        reset_future_.get(); reset_future_ = {};
        status_->setText("Reset acknowledged. Use Start Route when ready.");
      } else if (std::chrono::steady_clock::now() >= deadline_) {
        cancelReset(); status_->setText("Reset timed out; result unknown. Check route progress.");
      }
    } catch (const std::exception & e) {
      cancelReset(); status_->setText(QString::fromUtf8(e.what()));
    }
  }
  updateControls();
}

void RouteExecutionPanel::showProgress()
{
  if (!progress_) {
    state_->setText("-"); visit_->setText("-"); waypoint_->setText("-"); reason_->setText("-");
    return;
  }
  const auto & p = *progress_;
  QString state = QString::fromStdString(p.state);
  if (p.state == "startup_wait") {
    state = "Awaiting start";
  } else if (p.state == "running") {
    state = "Following route";
  } else if (p.state == "waiting_resume") {
    state = "Awaiting resume";
  } else if (p.state == "completed") {
    state = "Route completed";
  } else if (p.state == "blocked") {
    state = "Progress blocked";
  } else if (p.state == "empty") {
    state = "No route";
  } else if (p.state == "disabled") {
    state = "Route disabled";
  }
  state_->setText(state);
  const bool active = p.route_index >= 0 &&
    static_cast<uint32_t>(p.route_index) < p.route_size;
  const auto visit = active ? QString::number(static_cast<uint32_t>(p.route_index) + 1) :
    (p.state == "completed" || p.route_size == 0 ? QString::number(p.route_size) : "-");
  visit_->setText(visit + " / " + QString::number(p.route_size));
  waypoint_->setText(p.waypoint_name.empty() ? "-" : QString::fromStdString(p.waypoint_name));
  reason_->setText(p.reason.empty() ? "-" : QString::fromStdString(p.reason));
}

void RouteExecutionPanel::updateControls()
{
  const bool connected = progress_ && progress_subscription_ &&
    progress_subscription_->get_publisher_count() > 0;
  const auto server = QString::fromStdString(server_prefix_);
  connection_->setText(
    !progress_ ? "Waiting for route progress" :
    (connected ? "Connected: " + server : "No progress publisher: " + server +
    " (last state shown)"));
  const bool ready = connected && !reset_future_.valid();
  const bool active = progress_ && progress_->route_index >= 0 &&
    static_cast<uint32_t>(progress_->route_index) < progress_->route_size;
  const bool waiting = progress_ &&
    (progress_->state == "startup_wait" || progress_->state == "waiting_resume");
  resume_button_->setText(
    progress_ && progress_->state == "waiting_resume" ?
    "Resume Route" : "Start Route");
  resume_button_->setEnabled(
    ready && active && waiting && !awaiting_progress_ &&
    resume_publisher_ && resume_publisher_->get_subscription_count() > 0);
  reset_button_->setEnabled(ready && reset_client_ && reset_client_->service_is_ready());
}

void RouteExecutionPanel::resumeRoute()
{
  updateControls();
  if (!resume_button_->isEnabled()) {return;}
  try {
    std_msgs::msg::Bool command;
    command.data = true;
    resume_publisher_->publish(command);
    awaiting_progress_ = true;
    status_->setText("Start/resume sent. Awaiting route progress.");
  } catch (const std::exception & e) {
    status_->setText(QString::fromUtf8(e.what()));
  }
  updateControls();
}

void RouteExecutionPanel::resetRoute()
{
  updateControls();
  if (!reset_button_->isEnabled()) {return;}
  try {
    auto pending = reset_client_->async_send_request(std::make_shared<Reset::Request>());
    reset_request_id_ = pending.request_id;
    reset_future_ = pending.share();
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    // Resetting an already waiting first visit need not publish another progress message.
    awaiting_progress_ = progress_->state != "startup_wait";
    status_->setText("Reset requested. Waiting for response.");
  } catch (const std::exception & e) {
    status_->setText(QString::fromUtf8(e.what()));
  }
  updateControls();
}

void RouteExecutionPanel::cancelReset()
{
  if (reset_future_.valid()) {
    reset_client_->remove_pending_request(reset_request_id_);
    reset_future_ = {};
  }
}
}  // namespace nav_waypoint_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  nav_waypoint_rviz_plugin::RouteExecutionPanel,
  rviz_common::Panel)
