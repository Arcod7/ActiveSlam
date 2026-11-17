// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#ifndef VIEW_FOLLOW_RVIZ__VIEW_FOLLOW_PANEL_HPP_
#define VIEW_FOLLOW_RVIZ__VIEW_FOLLOW_PANEL_HPP_

#include <memory>

#include <QCheckBox>
#include <QLabel>
#include <QString>
#include <QTimer>

#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>

namespace view_follow_rviz
{

class ViewFollowPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ViewFollowPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void followToggled(bool follow);
  void headingToggled(bool heading);
  void onCurrentViewChanged();
  void refreshAvailability();

private:
  void applyView();
  void setTargetFrame(const QString & frame);

  QCheckBox * follow_checkbox_;
  QCheckBox * heading_checkbox_;
  QLabel * availability_label_;
  QTimer * availability_timer_;
  // View state to put back when following is switched off.
  QString restore_frame_;
  QString restore_view_class_;
  // Set while we swap the view controller ourselves, so the resulting
  // currentChanged() is not mistaken for the user picking a view.
  bool applying_view_ = false;
};

}  // namespace view_follow_rviz

#endif  // VIEW_FOLLOW_RVIZ__VIEW_FOLLOW_PANEL_HPP_
