// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include "motion_safety_rviz/motion_safety_panel.hpp"

#include <QHBoxLayout>
#include <QMessageBox>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace motion_safety_rviz
{

namespace
{
constexpr char kEnableTopic[] = "/motion/enable";
constexpr char kStatusTopic[] = "/motion/safety_status";
}  // namespace

MotionSafetyPanel::MotionSafetyPanel(QWidget * parent)
: rviz_common::Panel(parent),
  status_label_(new QLabel("WAITING FOR SAFETY GATE", this)),
  enable_button_(new QPushButton("ENABLE MOTION", this)),
  disable_button_(new QPushButton("DISABLE NOW", this))
{
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setObjectName("motion_status_label");
  status_label_->setMinimumHeight(28);
  status_label_->setWordWrap(true);
  status_label_->setMinimumWidth(210);  // keeps the longest status on one line
  status_label_->setToolTip("Status: /motion/safety_status");
  status_label_->setStyleSheet(
    "QLabel { background: #555; color: white; padding: 4px 8px; font-weight: bold; }");

  enable_button_->setObjectName("enable_motion_button");
  enable_button_->setEnabled(false);
  enable_button_->setMinimumHeight(28);
  // Safety wording lives in the enable confirmation dialog, not on the panel.
  enable_button_->setToolTip(
    "Opens the ROS command gate (publishes /motion/enable). Does not arm ArduSub — "
    "keep the hardware kill switch and a manual pilot ready.");
  enable_button_->setStyleSheet(
    "QPushButton { background: #287a38; color: white; font-weight: bold; padding: 4px 10px; }"
    "QPushButton:disabled { background: #555; color: #aaa; }");

  disable_button_->setObjectName("disable_motion_button");
  disable_button_->setMinimumHeight(28);
  disable_button_->setToolTip("Closes the ROS command gate. Does not disarm ArduSub.");
  disable_button_->setStyleSheet(
    "QPushButton { background: #a62222; color: white; font-weight: bold; padding: 4px 10px; }");

  auto * layout = new QHBoxLayout;
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(4);
  layout->addWidget(status_label_, 1);
  layout->addWidget(enable_button_);
  layout->addWidget(disable_button_);
  setLayout(layout);

  connect(enable_button_, &QPushButton::clicked, this, &MotionSafetyPanel::requestEnable);
  connect(disable_button_, &QPushButton::clicked, this, &MotionSafetyPanel::requestDisable);
  connect(
    this, &MotionSafetyPanel::statusReceived,
    this, &MotionSafetyPanel::updateStatus,
    Qt::QueuedConnection);
}

MotionSafetyPanel::~MotionSafetyPanel()
{
  status_subscription_.reset();
  if (enable_publisher_ && rclcpp::ok()) {
    publishEnable(false);
  }
}

void MotionSafetyPanel::onInitialize()
{
  const auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    showUnavailable("RVIZ ROS NODE UNAVAILABLE");
    return;
  }

  node_ = node_abstraction->get_raw_node();
  if (!node_) {
    showUnavailable("RVIZ ROS NODE UNAVAILABLE");
    return;
  }

  enable_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(kEnableTopic, 10);
  status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    kStatusTopic, 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      Q_EMIT statusReceived(QString::fromStdString(message->data));
    });
}

void MotionSafetyPanel::requestEnable()
{
  const auto answer = QMessageBox::warning(
    this,
    "Enable ROS motion gate",
    "The controller may command the vehicle immediately. Confirm the area is clear, "
    "the tether is managed, depth/heading hold are ready, and the pilot can take over.\n\n"
    "This does NOT arm ArduSub. Enable ROS motion?",
    QMessageBox::Yes | QMessageBox::Cancel,
    QMessageBox::Cancel);

  if (answer != QMessageBox::Yes) {
    return;
  }

  if (!publishEnable(true)) {
    return;
  }
  enable_button_->setEnabled(false);
  status_label_->setText("ENABLE REQUESTED — WAITING FOR GATE");
  status_label_->setStyleSheet(
    "QLabel { background: #9a6800; color: white; padding: 4px 8px; font-weight: bold; }");
}

void MotionSafetyPanel::requestDisable()
{
  if (!publishEnable(false)) {
    return;
  }
  enable_button_->setEnabled(false);
  status_label_->setText("DISABLE REQUESTED");
  status_label_->setStyleSheet(
    "QLabel { background: #a62222; color: white; padding: 4px 8px; font-weight: bold; }");
}

void MotionSafetyPanel::updateStatus(const QString & status)
{
  status_label_->setText(status);

  if (status == "ACTIVE") {
    enable_button_->setEnabled(false);
    status_label_->setStyleSheet(
      "QLabel { background: #287a38; color: white; padding: 4px 8px; font-weight: bold; }");
    return;
  }

  if (status == "DISABLED") {
    enable_button_->setEnabled(enable_publisher_ != nullptr);
    status_label_->setStyleSheet(
      "QLabel { background: #555; color: white; padding: 4px 8px; font-weight: bold; }");
    return;
  }

  // Any other status is a fail-closed condition. Require the operator to disable
  // first and wait for DISABLED before another enable attempt.
  enable_button_->setEnabled(false);
  status_label_->setStyleSheet(
    "QLabel { background: #9a6800; color: white; padding: 4px 8px; font-weight: bold; }");
}

bool MotionSafetyPanel::publishEnable(bool enabled)
{
  if (!enable_publisher_) {
    showUnavailable("SAFETY GATE CONTROL UNAVAILABLE");
    return false;
  }

  std_msgs::msg::Bool message;
  message.data = enabled;
  enable_publisher_->publish(message);
  return true;
}

void MotionSafetyPanel::showUnavailable(const QString & reason)
{
  enable_button_->setEnabled(false);
  status_label_->setText(reason);
  status_label_->setStyleSheet(
    "QLabel { background: #a62222; color: white; padding: 4px 8px; font-weight: bold; }");
}

}  // namespace motion_safety_rviz

PLUGINLIB_EXPORT_CLASS(motion_safety_rviz::MotionSafetyPanel, rviz_common::Panel)
