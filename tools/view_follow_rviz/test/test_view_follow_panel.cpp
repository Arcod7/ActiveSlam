// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include <QApplication>
#include <QCheckBox>
#include <QLabel>

#include <gtest/gtest.h>

#include "view_follow_rviz/view_follow_panel.hpp"

TEST(ViewFollowPanel, StartsOffAndHeadingTracksThePositionCheckbox)
{
  int argc = 1;
  char application_name[] = "view_follow_panel_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  view_follow_rviz::ViewFollowPanel panel;
  auto * follow = panel.findChild<QCheckBox *>("follow_ground_truth_checkbox");
  auto * heading = panel.findChild<QCheckBox *>("follow_heading_checkbox");
  auto * status_label = panel.findChild<QLabel *>("follow_frame_status_label");

  ASSERT_NE(follow, nullptr);
  ASSERT_NE(heading, nullptr);
  ASSERT_NE(status_label, nullptr);
  EXPECT_FALSE(follow->isChecked());
  EXPECT_FALSE(heading->isChecked());
  EXPECT_FALSE(heading->isEnabled());
  EXPECT_EQ(status_label->text(), "bluerov2/base_link_gt");

  // No DisplayContext outside RViz: toggling must be a no-op, not a crash.
  follow->setChecked(true);
  EXPECT_TRUE(heading->isEnabled());
  heading->setChecked(true);

  // Heading has no target frame to take yaw from once position is off.
  follow->setChecked(false);
  EXPECT_FALSE(heading->isChecked());
  EXPECT_FALSE(heading->isEnabled());
}
