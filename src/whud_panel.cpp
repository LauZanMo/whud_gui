#include "whud_panel.hpp"

namespace whud_gui {
whud_panel::whud_panel(QWidget* parent)
    : rviz::Panel(parent),
      ui_(new Ui::Widget),
      basic_control_(new whud_basic_control(this, ui_)) {
  ui_->setupUi(this);

  connect(
      ui_->BasicCmdComboBox,
      static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      basic_control_, &whud_basic_control::ChangeComboBox);

  connect(ui_->SendButton, &QPushButton::clicked, basic_control_,
          &whud_basic_control::ClickSendButton);
}

whud_panel::~whud_panel() {
  delete ui_;
  delete basic_control_;
}
}  // namespace whud_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_gui::whud_panel, rviz::Panel);
