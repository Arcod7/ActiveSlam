// Copyright 2026 Antoine Esman
// SPDX-License-Identifier: MIT

#include "legend_rviz/legend_panel.hpp"

#include <QFont>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QString>
#include <QVector>

#include <pluginlib/class_list_macros.hpp>

namespace legend_rviz
{

namespace
{

// Swatch shape, so a line entry does not read like a point entry.
enum class Shape
{
  Line,      // path / trajectory displays
  Dot,       // point clouds
  Sphere,    // goal marker
  Arrow,     // frontier and vehicle markers
  Ramp,      // two-ended colour scale
  Cube,      // map voxels
};

struct Entry
{
  Shape shape;
  const char * color;      // swatch colour, or ramp start
  const char * color_end;  // ramp end, nullptr otherwise
  const char * text;
};

struct Group
{
  const char * title;
  QVector<Entry> entries;
};

// Colours are duplicated from the publishers, so every one is sourced here:
//   demo.rviz .................. SLAM/dead-reckoning/planned-path line colours
//   visualizer.py .............. C_FRONTIER, C_PATH (goal sphere + path)
//   launcher.py ................ goto target sphere (same C_PATH brown)
//   benchmark.py ............... live position-error line
//   safety_gate.py ............. vehicle arrow state colours
//   wall_looking.py ............ SELECTED_WALL_RGBA
//   tsdf_mapper.py ............. _confidence_colormap endpoints, BAND_TINT
const QVector<Group> & legendGroups()
{
  static const QVector<Group> groups = {
    {"Trajectory", {
        {Shape::Line, "#1e78ff", nullptr, "SLAM estimate"},
        {Shape::Line, "#eb3c3c", nullptr, "Dead reckoning"},
        {Shape::Line, "#965a28", nullptr, "Planned path (A*)"},
        {Shape::Line, "#ffd900", nullptr, "Position error (SLAM vs truth)"},
      }},
    {"Goals & sensing", {
        {Shape::Arrow, "#00a8c1", nullptr, "Frontier, pointing into unknown"},
        {Shape::Sphere, "#965a28", nullptr, "Current goal (frontier or goto)"},
        {Shape::Dot, "#ffffff", nullptr, "Sonar returns (/cloud_in)"},
        {Shape::Line, "#338cff", nullptr, "Wall voxel the heading is taken from"},
      }},
    {"Vehicle arrow (safety gate)", {
        {Shape::Arrow, "#29db29", nullptr, "Motion enabled"},
        {Shape::Arrow, "#b333e6", nullptr, "Gate disabled"},
        {Shape::Arrow, "#00cce6", nullptr, "Revisiting a past view"},
        {Shape::Arrow, "#ffffff", nullptr, "Initial scan sweep"},
      }},
    {"Map voxels (TSDF)", {
        {Shape::Ramp, "#ff8c00", "#1ae633", "Hue: times observed, low to high"},
        {Shape::Ramp, "#94bd99", "#1ae633", "Saturation: wall confidence, pale = borderline"},
        {Shape::Cube, "#4f7598", nullptr, "Blue tint: inside the planner's Z band"},
        {Shape::Dot, "#28dc28", nullptr, "Ground-truth map"},
      }},
  };
  return groups;
}

QLabel * makeSwatch(const Entry & entry, QWidget * parent)
{
  auto * swatch = new QLabel(parent);
  swatch->setAlignment(Qt::AlignCenter);

  switch (entry.shape) {
    case Shape::Line:
      swatch->setFixedSize(22, 4);
      swatch->setStyleSheet(QString("QLabel { background: %1; }").arg(QString(entry.color)));
      break;
    case Shape::Dot:
      swatch->setFixedSize(9, 9);
      swatch->setStyleSheet(
        QString("QLabel { background: %1; border-radius: 4px; }").arg(QString(entry.color)));
      break;
    case Shape::Sphere:
      swatch->setFixedSize(15, 15);
      swatch->setStyleSheet(
        QString("QLabel { background: %1; border-radius: 7px; }").arg(QString(entry.color)));
      break;
    case Shape::Arrow:
      swatch->setFixedSize(22, 15);
      swatch->setText(QString::fromUtf8("\xe2\x9e\x9c"));  // heavy round-tipped arrow
      swatch->setStyleSheet(
        QString("QLabel { color: %1; background: transparent; }").arg(QString(entry.color)));
      break;
    case Shape::Cube:
      swatch->setFixedSize(11, 11);
      swatch->setStyleSheet(
        QString("QLabel { background: %1; }").arg(QString(entry.color)));
      break;
    case Shape::Ramp:
      swatch->setFixedSize(38, 11);
      swatch->setStyleSheet(
        QString(
          "QLabel { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
          "stop:0 %1, stop:1 %2); }")
        .arg(QString(entry.color), QString(entry.color_end)));
      break;
  }
  return swatch;
}

}  // namespace

LegendPanel::LegendPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setObjectName("legend_panel");

  QFont text_font;
  const int base_point_size = text_font.pointSize() > 0 ? text_font.pointSize() : 10;
  text_font.setPointSize(base_point_size - 1);
  QFont title_font(text_font);
  title_font.setBold(true);

  auto * layout = new QGridLayout;
  layout->setContentsMargins(8, 4, 8, 4);
  layout->setHorizontalSpacing(6);
  layout->setVerticalSpacing(2);

  int column = 0;
  for (const auto & group : legendGroups()) {
    auto * title = new QLabel(group.title, this);
    title->setFont(title_font);
    title->setStyleSheet("QLabel { color: #d0d0d0; }");
    layout->addWidget(title, 0, column, 1, 2);

    int row = 1;
    for (const auto & entry : group.entries) {
      auto * text = new QLabel(entry.text, this);
      text->setFont(text_font);
      text->setStyleSheet("QLabel { color: #b0b0b0; }");
      layout->addWidget(makeSwatch(entry, this), row, column, Qt::AlignCenter);
      layout->addWidget(text, row, column + 1);
      ++row;
    }
    // Third column is an empty gutter: group titles are wider than their
    // swatch+text pair and would otherwise run into the next group.
    layout->setColumnMinimumWidth(column + 2, 26);
    column += 3;
  }
  layout->setColumnStretch(column, 1);
  layout->setRowStretch(layout->rowCount(), 1);
  setLayout(layout);
}

void LegendPanel::onInitialize()
{
  // Corner ownership is a QMainWindow property, not something RViz stores in
  // the .rviz file, so it has to be set from code once the panel is docked.
  // Without it a bottom dock spans the full window width and this legend runs
  // under the Displays pane, far from the view it describes.
  if (auto * frame = qobject_cast<QMainWindow *>(window())) {
    frame->setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  }
}

}  // namespace legend_rviz

PLUGINLIB_EXPORT_CLASS(legend_rviz::LegendPanel, rviz_common::Panel)
