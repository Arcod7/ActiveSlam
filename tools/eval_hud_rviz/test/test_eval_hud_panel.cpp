// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include <QApplication>
#include <QColor>
#include <QLabel>
#include <QMetaObject>
#include <QString>

#include <gtest/gtest.h>

#include "eval_hud_rviz/eval_hud_panel.hpp"

namespace
{
struct PanelFixture
{
  eval_hud_rviz::EvalHudPanel panel;
  QLabel * state_label = panel.findChild<QLabel *>("eval_hud_state_label");
  QLabel * metrics_label = panel.findChild<QLabel *>("eval_hud_metrics_label");
  QLabel * viz_cap_label = panel.findChild<QLabel *>("eval_hud_viz_cap_label");
};
}  // namespace

TEST(EvalHudPanel, StartsWithPlaceholdersAndUpdatesOnMetrics)
{
  int argc = 1;
  char application_name[] = "eval_hud_panel_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  PanelFixture fixture;
  ASSERT_NE(fixture.metrics_label, nullptr);
  ASSERT_NE(fixture.state_label, nullptr);
  EXPECT_EQ(fixture.metrics_label->text(), "no eval data yet (requires slam:=slam)");
  EXPECT_EQ(fixture.state_label->text(), "WAITING FOR MOTION SAFETY GATE");

  const QString sample_text =
    "err 0.12 m  |  ATE 0.34 m  |  RPE 0.05 m / 1.2 deg\n"
    "KF 7  |  LC 1  |  D-opt 0.0080  |  ANEES 1.2";
  ASSERT_TRUE(
    QMetaObject::invokeMethod(
      &fixture.panel, "updateMetrics", Q_ARG(QString, sample_text)));
  EXPECT_EQ(fixture.metrics_label->text(), sample_text);
}

TEST(EvalHudPanel, MotionStateRowTakesTheMarkerTextAndColour)
{
  int argc = 1;
  char application_name[] = "eval_hud_panel_state_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  PanelFixture fixture;
  ASSERT_NE(fixture.state_label, nullptr);

  const QColor cyan = QColor::fromRgbF(0.0, 0.8, 0.9);
  ASSERT_TRUE(
    QMetaObject::invokeMethod(
      &fixture.panel, "updateMotionState",
      Q_ARG(QString, "REVISITING"), Q_ARG(QColor, cyan)));

  EXPECT_EQ(fixture.state_label->text(), "REVISITING");
  // The arrow colour has to reach the stylesheet, otherwise the row and the
  // marker can disagree.
  EXPECT_TRUE(fixture.state_label->styleSheet().contains(cyan.name()));
}

TEST(EvalHudPanel, VoxelCapRowOnlyShowsWhileThereIsAWarning)
{
  int argc = 1;
  char application_name[] = "eval_hud_panel_viz_cap_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  PanelFixture fixture;
  ASSERT_NE(fixture.viz_cap_label, nullptr);
  EXPECT_TRUE(fixture.viz_cap_label->isHidden());

  const QString warning = "VOXEL VIEW CAPPED - showing 100000 of 250000 (40%)";
  ASSERT_TRUE(
    QMetaObject::invokeMethod(
      &fixture.panel, "updateVizCap", Q_ARG(QString, warning)));
  EXPECT_EQ(fixture.viz_cap_label->text(), warning);
  EXPECT_FALSE(fixture.viz_cap_label->isHidden());

  // An empty payload is how tsdf_mapper clears the cap, so the row must go.
  ASSERT_TRUE(
    QMetaObject::invokeMethod(
      &fixture.panel, "updateVizCap", Q_ARG(QString, QString())));
  EXPECT_TRUE(fixture.viz_cap_label->isHidden());
}
