// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#ifndef EVAL_HUD_RVIZ__EVAL_HUD_PANEL_HPP_
#define EVAL_HUD_RVIZ__EVAL_HUD_PANEL_HPP_

#include <QLabel>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace eval_hud_rviz
{

class EvalHudPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit EvalHudPanel(QWidget * parent = nullptr);
  ~EvalHudPanel() override;

  void onInitialize() override;

Q_SIGNALS:
  void metricsReceived(const QString & text);

private Q_SLOTS:
  void updateMetrics(const QString & text);

private:
  QLabel * metrics_label_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_subscription_;
};

}  // namespace eval_hud_rviz

#endif  // EVAL_HUD_RVIZ__EVAL_HUD_PANEL_HPP_
