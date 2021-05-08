#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

#include "ui_widget.h"
#include "whud_basic_control.hpp"

namespace Ui {
class Widget;
}

namespace whud_gui {
class whud_panel : public rviz::Panel {
  Q_OBJECT

 public:
  whud_panel(QWidget* parent = nullptr);
  ~whud_panel();

  // virtual void load(const rviz::Config& config);
  // virtual void save(rviz::Config config) const;

 protected:
  Ui::Widget* ui_ = nullptr;
  whud_basic_control* basic_control_ = nullptr;
};
}  // namespace whud_gui
