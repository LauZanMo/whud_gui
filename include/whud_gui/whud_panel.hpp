#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

#include "ui_widget.h"

namespace Ui {
class Widget;
}

namespace whud_gui {
class whud_panel : public rviz::Panel {
  Q_OBJECT

 public:
  whud_panel(QWidget* parent = 0);

  // virtual void load(const rviz::Config& config);
  // virtual void save(rviz::Config config) const;

 protected:
  Ui::Widget* ui;
};
}  // namespace whud_gui
