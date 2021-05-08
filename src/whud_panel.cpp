#include "whud_panel.hpp"

namespace whud_gui {
whud_panel::whud_panel(QWidget* parent)
    : rviz::Panel(parent), ui(new Ui::Widget) {
  ui->setupUi(this);
}
}  // namespace whud_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_gui::whud_panel, rviz::Panel);
