// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#ifndef MOTION_SAFETY_RVIZ__MOTION_SAFETY_PANEL_HPP_
#define MOTION_SAFETY_RVIZ__MOTION_SAFETY_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace motion_safety_rviz
{

class MotionSafetyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MotionSafetyPanel(QWidget * parent = nullptr);
  ~MotionSafetyPanel() override;

  void onInitialize() override;

Q_SIGNALS:
  void statusReceived(const QString & status);

private Q_SLOTS:
  void requestEnable();
  void requestDisable();
  void updateStatus(const QString & status);

private:
  bool publishEnable(bool enabled);
  void showUnavailable(const QString & reason);

  QLabel * status_label_;
  QPushButton * enable_button_;
  QPushButton * disable_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
};

}  // namespace motion_safety_rviz

#endif  // MOTION_SAFETY_RVIZ__MOTION_SAFETY_PANEL_HPP_
