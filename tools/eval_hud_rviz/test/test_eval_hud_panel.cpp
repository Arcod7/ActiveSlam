// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include <QApplication>
#include <QLabel>
#include <QMetaObject>
#include <QString>

#include <gtest/gtest.h>

#include "eval_hud_rviz/eval_hud_panel.hpp"

TEST(EvalHudPanel, StartsWithPlaceholderAndUpdatesOnMetrics)
{
  int argc = 1;
  char application_name[] = "eval_hud_panel_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  eval_hud_rviz::EvalHudPanel panel;
  auto * metrics_label = panel.findChild<QLabel *>("eval_hud_metrics_label");

  ASSERT_NE(metrics_label, nullptr);
  EXPECT_EQ(metrics_label->text(), "no eval data yet (requires slam:=slam)");

  const QString sample_text = "err 0.12m | ATE 0.34m | RPE 0.05m/1.2deg\nKF 7 | LC 1 | D-opt 0.0080";
  ASSERT_TRUE(
    QMetaObject::invokeMethod(
      &panel, "updateMetrics", Q_ARG(QString, sample_text)));
  EXPECT_EQ(metrics_label->text(), sample_text);
}
