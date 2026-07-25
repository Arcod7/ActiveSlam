// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include "eval_hud_rviz/eval_hud_panel.hpp"

#include <QFont>
#include <QHBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace eval_hud_rviz
{

namespace
{
constexpr char kMarkersTopic[] = "/eval/markers";
constexpr char kHudNamespace[] = "eval_hud";
constexpr char kPlaceholderText[] = "no eval data yet (requires slam:=slam)";
}  // namespace

EvalHudPanel::EvalHudPanel(QWidget * parent)
: rviz_common::Panel(parent),
  metrics_label_(new QLabel(kPlaceholderText, this))
{
  QFont label_font("monospace");
  label_font.setStyleHint(QFont::TypeWriter);
  metrics_label_->setFont(label_font);
  metrics_label_->setAlignment(Qt::AlignCenter);
  metrics_label_->setObjectName("eval_hud_metrics_label");
  metrics_label_->setStyleSheet(
    "QLabel { background: #222; color: #e0e0e0; padding: 4px; }");

  auto * layout = new QHBoxLayout;
  layout->addWidget(metrics_label_);
  layout->setContentsMargins(4, 2, 4, 2);
  setLayout(layout);

  connect(
    this, &EvalHudPanel::metricsReceived,
    this, &EvalHudPanel::updateMetrics,
    Qt::QueuedConnection);
}

EvalHudPanel::~EvalHudPanel() = default;

void EvalHudPanel::onInitialize()
{
  const auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    metrics_label_->setText("RVIZ ROS NODE UNAVAILABLE");
    return;
  }

  node_ = node_abstraction->get_raw_node();
  if (!node_) {
    metrics_label_->setText("RVIZ ROS NODE UNAVAILABLE");
    return;
  }

  markers_subscription_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    kMarkersTopic, 10,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr message) {
      for (const auto & marker : message->markers) {
        if (marker.ns == kHudNamespace) {
          Q_EMIT metricsReceived(QString::fromStdString(marker.text));
          return;
        }
      }
    });
}

void EvalHudPanel::updateMetrics(const QString & text)
{
  metrics_label_->setText(text);
}

}  // namespace eval_hud_rviz

PLUGINLIB_EXPORT_CLASS(eval_hud_rviz::EvalHudPanel, rviz_common::Panel)
