// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include <QApplication>
#include <QDockWidget>
#include <QLabel>
#include <QList>
#include <QMainWindow>

#include <gtest/gtest.h>

#include "legend_rviz/legend_panel.hpp"

TEST(LegendPanel, ListsEveryEntryWithASwatch)
{
  int argc = 1;
  char application_name[] = "legend_panel_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  legend_rviz::LegendPanel panel;
  const auto labels = panel.findChildren<QLabel *>();

  // 4 group titles + 16 entries, each entry contributing a swatch and a text label.
  EXPECT_EQ(labels.size(), 4 + 2 * 16);

  int with_text = 0;
  for (const auto * label : labels) {
    if (!label->text().isEmpty()) {
      ++with_text;
    }
    EXPECT_FALSE(label->styleSheet().isEmpty());
  }
  // Arrow swatches carry a glyph, so the text-bearing labels are titles +
  // descriptions + the 5 arrows.
  EXPECT_EQ(with_text, 4 + 16 + 5);
}

TEST(LegendPanel, KeepsTheBottomStripOutFromUnderTheLeftDock)
{
  int argc = 1;
  char application_name[] = "legend_panel_corner_test";
  char * argv[] = {application_name, nullptr};
  QApplication application(argc, argv);

  QMainWindow frame;
  auto * dock = new QDockWidget(&frame);
  auto * panel = new legend_rviz::LegendPanel;
  dock->setWidget(panel);
  frame.addDockWidget(Qt::BottomDockWidgetArea, dock);

  // Qt's default hands both bottom corners to the bottom area, which is what
  // makes a full-width legend run under the Displays pane.
  ASSERT_EQ(frame.corner(Qt::BottomLeftCorner), Qt::BottomDockWidgetArea);

  panel->onInitialize();

  EXPECT_EQ(frame.corner(Qt::BottomLeftCorner), Qt::LeftDockWidgetArea);
}
