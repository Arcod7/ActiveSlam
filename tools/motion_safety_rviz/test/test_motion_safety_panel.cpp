// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include <QApplication>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>

#include <gtest/gtest.h>

#include "motion_safety_rviz/motion_safety_panel.hpp"

TEST(MotionSafetyPanel, StartsFailClosedAndDisableRemainsAvailable)
{
  int argc = 1;
  char application_name[] = "motion_safety_panel_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  motion_safety_rviz::MotionSafetyPanel panel;
  auto * enable_button = panel.findChild<QPushButton *>("enable_motion_button");
  auto * disable_button = panel.findChild<QPushButton *>("disable_motion_button");
  auto * status_label = panel.findChild<QLabel *>("motion_status_label");

  ASSERT_NE(enable_button, nullptr);
  ASSERT_NE(disable_button, nullptr);
  ASSERT_NE(status_label, nullptr);
  EXPECT_FALSE(enable_button->isEnabled());
  EXPECT_TRUE(disable_button->isEnabled());
  EXPECT_EQ(status_label->text(), "WAITING FOR SAFETY GATE");

  ASSERT_TRUE(QMetaObject::invokeMethod(&panel, "requestDisable"));
  EXPECT_FALSE(enable_button->isEnabled());
  EXPECT_EQ(status_label->text(), "SAFETY GATE CONTROL UNAVAILABLE");
}
