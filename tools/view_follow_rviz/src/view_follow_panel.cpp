// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include "view_follow_rviz/view_follow_panel.hpp"

#include <string>

#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

namespace view_follow_rviz
{

namespace
{
// Ground-truth vehicle frame, broadcast by odom_tf_sync under slam:=slam only.
constexpr char kGroundTruthFrame[] = "bluerov2/base_link_gt";
constexpr char kTargetFrameProperty[] = "Target Frame";
constexpr char kInvertZProperty[] = "Invert Z Axis";
// Target Frame alone only tracks the frame's position; this view also turns
// with its yaw.
constexpr char kFollowerViewClass[] = "rviz_default_plugins/ThirdPersonFollower";
constexpr char kDefaultViewClass[] = "rviz_default_plugins/Orbit";
constexpr int kAvailabilityPollMs = 1000;
}  // namespace

ViewFollowPanel::ViewFollowPanel(QWidget * parent)
: rviz_common::Panel(parent),
  follow_checkbox_(new QCheckBox("Follow position", this)),
  heading_checkbox_(new QCheckBox("Follow heading", this)),
  availability_label_(new QLabel(kGroundTruthFrame, this)),
  availability_timer_(new QTimer(this)),
  restore_frame_(rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING),
  restore_view_class_(kDefaultViewClass)
{
  follow_checkbox_->setObjectName("follow_ground_truth_checkbox");
  follow_checkbox_->setToolTip(
    QString(
      "Locks the camera onto %1, the simulator's exact vehicle pose. "
      "That frame is only broadcast under slam:=slam; under slam:=none the "
      "fixed-frame view already shows ground truth.").arg(kGroundTruthFrame));

  heading_checkbox_->setObjectName("follow_heading_checkbox");
  heading_checkbox_->setEnabled(false);
  heading_checkbox_->setToolTip(
    "Also turns the camera with the vehicle's yaw, by switching the view to "
    "ThirdPersonFollower. The horizon stays level — roll and pitch do not "
    "tilt the scene. Clearing it returns the view to its previous type.");

  availability_label_->setObjectName("follow_frame_status_label");
  availability_label_->setStyleSheet("QLabel { color: #aaa; }");

  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(2);
  layout->addWidget(follow_checkbox_);
  layout->addWidget(heading_checkbox_);
  layout->addWidget(availability_label_);
  setLayout(layout);

  connect(follow_checkbox_, &QCheckBox::toggled, this, &ViewFollowPanel::followToggled);
  connect(heading_checkbox_, &QCheckBox::toggled, this, &ViewFollowPanel::headingToggled);
  connect(availability_timer_, &QTimer::timeout, this, &ViewFollowPanel::refreshAvailability);
}

void ViewFollowPanel::onInitialize()
{
  auto * context = getDisplayContext();
  if (!context) {
    return;
  }

  if (auto * view_manager = context->getViewManager()) {
    if (auto * view = view_manager->getCurrent()) {
      restore_view_class_ = view->getClassId();
    }
    // The Views panel can swap the controller under us; re-apply on every change.
    connect(
      view_manager, &rviz_common::ViewManager::currentChanged,
      this, &ViewFollowPanel::onCurrentViewChanged);
  }

  availability_timer_->start(kAvailabilityPollMs);
  refreshAvailability();

  if (follow_checkbox_->isChecked()) {
    applyView();
  }
}

void ViewFollowPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Follow", follow_checkbox_->isChecked());
  config.mapSetValue("FollowHeading", heading_checkbox_->isChecked());
}

void ViewFollowPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  bool follow = false;
  if (config.mapGetBool("Follow", &follow)) {
    follow_checkbox_->setChecked(follow);
  }
  bool heading = false;
  if (config.mapGetBool("FollowHeading", &heading)) {
    heading_checkbox_->setChecked(heading);
  }
  // RViz loads the Views config after the panels, so a view swapped in from
  // here is overwritten by it. Re-apply once the whole load has settled.
  QTimer::singleShot(0, this, [this]() {applyView();});
}

void ViewFollowPanel::followToggled(bool follow)
{
  heading_checkbox_->setEnabled(follow);
  if (!follow) {
    // Heading has no meaning without a target frame to take it from.
    heading_checkbox_->setChecked(false);
  }
  applyView();
  Q_EMIT configChanged();
}

void ViewFollowPanel::headingToggled(bool)
{
  applyView();
  Q_EMIT configChanged();
}

void ViewFollowPanel::onCurrentViewChanged()
{
  if (applying_view_) {
    return;
  }
  auto * context = getDisplayContext();
  if (context && context->getViewManager()) {
    if (auto * view = context->getViewManager()->getCurrent()) {
      restore_view_class_ = view->getClassId();
    }
  }
  if (follow_checkbox_->isChecked()) {
    applyView();
  }
}

void ViewFollowPanel::refreshAvailability()
{
  auto * context = getDisplayContext();
  if (!context || !context->getFrameManager()) {
    return;
  }

  std::string error;
  const bool missing =
    context->getFrameManager()->frameHasProblems(kGroundTruthFrame, error);
  availability_label_->setText(
    missing ?
    QString("%1 not published").arg(kGroundTruthFrame) :
    QString(kGroundTruthFrame));
  availability_label_->setStyleSheet(
    missing ? "QLabel { color: #c08a00; }" : "QLabel { color: #aaa; }");
}

void ViewFollowPanel::applyView()
{
  auto * context = getDisplayContext();
  if (!context || !context->getViewManager()) {
    return;
  }
  auto * view_manager = context->getViewManager();
  auto * view = view_manager->getCurrent();
  if (!view) {
    return;
  }

  const bool follow = follow_checkbox_->isChecked();
  const bool heading = follow && heading_checkbox_->isChecked();
  const QString wanted_class = heading ? kFollowerViewClass : restore_view_class_;

  if (view->getClassId() != wanted_class) {
    // mimic() does not carry Invert Z Axis across a type switch, and the
    // fixed frame is NED — without it the new view comes up upside down.
    const auto * invert_z = view->subProp(kInvertZProperty);
    const QVariant invert_z_value = invert_z ? invert_z->getValue() : QVariant();

    applying_view_ = true;
    view_manager->setCurrentViewControllerType(wanted_class);
    applying_view_ = false;
    // The old controller is gone; the switch built a fresh one.
    view = view_manager->getCurrent();
    if (!view) {
      return;
    }
    if (invert_z_value.isValid()) {
      if (auto * target_invert_z = view->subProp(kInvertZProperty)) {
        target_invert_z->setValue(invert_z_value);
      }
    }
  }

  setTargetFrame(follow ? QString(kGroundTruthFrame) : restore_frame_);
}

void ViewFollowPanel::setTargetFrame(const QString & frame)
{
  auto * context = getDisplayContext();
  if (!context || !context->getViewManager()) {
    return;
  }
  auto * view = context->getViewManager()->getCurrent();
  if (!view) {
    return;
  }

  auto * target = view->subProp(kTargetFrameProperty);
  if (!target) {
    return;
  }

  const QString current = target->getValue().toString();
  if (current == frame) {
    return;
  }
  if (frame == kGroundTruthFrame) {
    restore_frame_ = current;
  }
  target->setValue(frame);
}

}  // namespace view_follow_rviz

PLUGINLIB_EXPORT_CLASS(view_follow_rviz::ViewFollowPanel, rviz_common::Panel)
