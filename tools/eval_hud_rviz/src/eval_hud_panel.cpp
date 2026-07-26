// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include "eval_hud_rviz/eval_hud_panel.hpp"

#include <algorithm>

#include <QFont>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace eval_hud_rviz
{

namespace
{
constexpr char kMarkersTopic[] = "/eval/markers";
constexpr char kHudNamespace[] = "eval_hud";
constexpr char kRobotMarkerTopic[] = "/motion/robot_marker";
constexpr char kMotionStateNamespace[] = "motion_state_text";
constexpr char kPlaceholderText[] = "no eval data yet (requires slam:=slam)";
constexpr char kStatePlaceholderText[] = "WAITING FOR MOTION SAFETY GATE";
constexpr char kStatePlaceholderColor[] = "#888888";

// The state row carries the same colour as the vehicle arrow, so the two read
// as one signal: coloured text on a tinted strip, keyed by the left border.
QString stateStyleSheet(const QString & color_name)
{
  return QString(
    "QLabel { background: #1a1a1a; color: %1; border-left: 5px solid %1; "
    "padding: 5px 8px; font-weight: bold; letter-spacing: 1px; }").arg(color_name);
}

QColor markerColor(const visualization_msgs::msg::Marker & marker)
{
  const auto clamp01 = [](float v) {return std::clamp(v, 0.0f, 1.0f);};
  return QColor::fromRgbF(
    clamp01(marker.color.r), clamp01(marker.color.g), clamp01(marker.color.b));
}
}  // namespace

EvalHudPanel::EvalHudPanel(QWidget * parent)
: rviz_common::Panel(parent),
  state_label_(new QLabel(kStatePlaceholderText, this)),
  metrics_label_(new QLabel(kPlaceholderText, this))
{
  QFont label_font("monospace");
  label_font.setStyleHint(QFont::TypeWriter);

  QFont state_font(label_font);
  // pointSize() is -1 for a pixel-sized font, which would shrink the row.
  const int base_point_size = label_font.pointSize() > 0 ? label_font.pointSize() : 10;
  state_font.setPointSize(base_point_size + 2);
  state_label_->setFont(state_font);
  state_label_->setAlignment(Qt::AlignCenter);
  state_label_->setObjectName("eval_hud_state_label");
  state_label_->setToolTip("Vehicle state: /motion/robot_marker, same colour as the arrow");
  state_label_->setStyleSheet(stateStyleSheet(kStatePlaceholderColor));

  metrics_label_->setFont(label_font);
  metrics_label_->setAlignment(Qt::AlignCenter);
  metrics_label_->setObjectName("eval_hud_metrics_label");
  metrics_label_->setToolTip("SLAM metrics: /eval/markers");
  metrics_label_->setStyleSheet(
    "QLabel { background: #222; color: #e0e0e0; padding: 6px 8px; }");

  // No spacing between the rows: one block, state on top.
  auto * layout = new QVBoxLayout;
  layout->addWidget(state_label_);
  layout->addWidget(metrics_label_);
  layout->setContentsMargins(4, 2, 4, 2);
  layout->setSpacing(0);
  setLayout(layout);

  connect(
    this, &EvalHudPanel::metricsReceived,
    this, &EvalHudPanel::updateMetrics,
    Qt::QueuedConnection);
  connect(
    this, &EvalHudPanel::motionStateReceived,
    this, &EvalHudPanel::updateMotionState,
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

  robot_marker_subscription_ = node_->create_subscription<visualization_msgs::msg::Marker>(
    kRobotMarkerTopic, 10,
    [this](const visualization_msgs::msg::Marker::SharedPtr message) {
      if (message->ns != kMotionStateNamespace) {
        return;  // the arrow marker shares this topic
      }
      Q_EMIT motionStateReceived(
        QString::fromStdString(message->text), markerColor(*message));
    });
}

void EvalHudPanel::updateMetrics(const QString & text)
{
  metrics_label_->setText(text);
}

void EvalHudPanel::updateMotionState(const QString & text, const QColor & color)
{
  state_label_->setText(text);
  state_label_->setStyleSheet(stateStyleSheet(color.name()));
}

}  // namespace eval_hud_rviz

PLUGINLIB_EXPORT_CLASS(eval_hud_rviz::EvalHudPanel, rviz_common::Panel)
