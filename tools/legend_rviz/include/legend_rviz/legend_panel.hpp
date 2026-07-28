// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#ifndef LEGEND_RVIZ__LEGEND_PANEL_HPP_
#define LEGEND_RVIZ__LEGEND_PANEL_HPP_

#include <QWidget>

#include <rviz_common/panel.hpp>

namespace legend_rviz
{

/// Static key for the demo.rviz displays. Holds no ROS state — the colours are
/// fixed by the publishers, so nothing here can go stale at runtime.
class LegendPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LegendPanel(QWidget * parent = nullptr);

  /// Hands the bottom-left corner to the left dock, so this panel spans only
  /// the render area rather than running under the Displays pane.
  void onInitialize() override;
};

}  // namespace legend_rviz

#endif  // LEGEND_RVIZ__LEGEND_PANEL_HPP_
