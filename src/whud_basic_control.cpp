#include "whud_basic_control.hpp"

whud_basic_control::whud_basic_control(QObject* parent, Ui::Widget* ui)
    : QObject(parent),
      ui_(ui),
      nh_(""),
      take_off_validator_1_(new QDoubleValidator(0.1, 99, 1)),
      take_off_validator_2_(new QDoubleValidator(0.1, 9, 1)),
      land_validator_1_(new QDoubleValidator(-9, -0.1, 1)),
      set_height_validator_1_(new QDoubleValidator(-99, +99, 1)),
      set_height_validator_2_(new QDoubleValidator(0.1, 9, 1)),
      set_yaw_validator_1_(new QDoubleValidator(0.1, 9, 1)),
      set_yaw_validator_2_(new QIntValidator(0, 1)) {
  SetNotation();
}

whud_basic_control::~whud_basic_control() {
  delete take_off_validator_1_;
  delete take_off_validator_2_;
  delete land_validator_1_;
  delete set_height_validator_1_;
  delete set_height_validator_2_;
  delete set_yaw_validator_1_;
  delete set_yaw_validator_2_;
}

void whud_basic_control::ChangeComboBox(int index) {
  switch (index) {
    // take off
    case 1:
      SetParamLabel("Height(m)", "Speed(m/s)");
      ClearParamEditor();
      SetParamValidator(take_off_validator_1_, take_off_validator_2_);
      break;
    // land
    case 2:
      SetParamLabel("Speed(m/s)", "None");
      ClearParamEditor();
      SetParamValidator(land_validator_1_);
      break;
    // set height
    case 3:
      SetParamLabel("Relative Height(m)", "Speed(m/s)");
      ClearParamEditor();
      SetParamValidator(set_height_validator_1_, set_height_validator_2_);
      break;
    // set yaw
    case 4:
      SetParamLabel("Angle(rad)", "Type");
      ClearParamEditor();
      SetParamValidator(set_yaw_validator_1_, set_yaw_validator_2_);
      break;
    default:
      break;
  }
}

void whud_basic_control::ClickSendButton() {
  ui_->SendButton->setEnabled(false);
  SendBasicCmd(ui_->BasicCmdComboBox->currentIndex());
  ui_->SendButton->setEnabled(true);
}

void whud_basic_control::SetNotation() {
  take_off_validator_1_->setNotation(QDoubleValidator::StandardNotation);
  take_off_validator_2_->setNotation(QDoubleValidator::StandardNotation);
  land_validator_1_->setNotation(QDoubleValidator::StandardNotation);
  set_height_validator_1_->setNotation(QDoubleValidator::StandardNotation);
  set_height_validator_2_->setNotation(QDoubleValidator::StandardNotation);
  set_yaw_validator_1_->setNotation(QDoubleValidator::StandardNotation);
}

void whud_basic_control::SetParamLabel(const char* label1, const char* label2) {
  ui_->BasicCmdParam1->setText(label1);
  ui_->BasicCmdParam2->setText(label2);
}

void whud_basic_control::ClearParamEditor() {
  ui_->Param1Editor->clear();
  ui_->Param2Editor->clear();
}

void whud_basic_control::SetParamValidator(QDoubleValidator* validator1,
                                           QDoubleValidator* validator2) {
  ui_->Param1Editor->setValidator(validator1);
  ui_->Param2Editor->setValidator(validator2);
}

void whud_basic_control::SetParamValidator(QDoubleValidator* validator) {
  ui_->Param1Editor->setValidator(validator);
  ui_->Param2Editor->setValidator(nullptr);
}

void whud_basic_control::SetParamValidator(QDoubleValidator* validator1,
                                           QIntValidator* validator2) {
  ui_->Param1Editor->setValidator(validator1);
  ui_->Param2Editor->setValidator(validator2);
}

bool whud_basic_control::ParamCheck(int num) {
  switch (num) {
    case 0:
      return true;
    case 1:
      if (ui_->Param1Editor->text().isEmpty())
        return false;
      else
        return true;
    case 2:
      if (ui_->Param1Editor->text().isEmpty() ||
          ui_->Param2Editor->text().isEmpty())
        return false;
      else
        return true;
    default:
      return false;
  }
}

void whud_basic_control::SendBasicCmd(int index) {
  uint8_t send_flag = 0;
  int ack_index = -1;
  int ack_result = -1;
  switch (index) {
    // take off
    case 1:
      if (!ParamCheck(2))
        break;
      else {
        send_flag++;
        break;
      }
    // land
    case 2:
      if (!ParamCheck(1))
        break;
      else {
        send_flag++;
        break;
      }
    // set height
    case 3:
      if (!ParamCheck(2))
        break;
      else {
        send_flag++;
        break;
      }
    // set yaw
    case 4:
      if (!ParamCheck(2))
        break;
      else {
        send_flag++;
        break;
      }
    default:
      send_flag = 3;
      break;
  }

  if (send_flag == 0) {
    ui_->OutputBrowser->setTextColor(QColor("red"));
    ui_->OutputBrowser->insertPlainText("Please complete all parameters!\n");
    ui_->OutputBrowser->moveCursor(QTextCursor::End);
    return;
  } else if (send_flag == 1) {
    ui_->OutputBrowser->setTextColor(QColor("red"));
    ui_->OutputBrowser->insertPlainText(
        "No ack from ACFLY, Maybe is disconncted!\n");
    ui_->OutputBrowser->moveCursor(QTextCursor::End);
    return;
  } else if (send_flag == 2) {
    ui_->OutputBrowser->setTextColor(QColor("green"));
    ui_->OutputBrowser->insertPlainText(
        "Send " + ui_->BasicCmdComboBox->currentText() + " messages OK!\n");
    ui_->OutputBrowser->moveCursor(QTextCursor::End);
  } else {
    ui_->OutputBrowser->setTextColor(QColor("red"));
    ui_->OutputBrowser->insertPlainText("Please choose command!\n");
    ui_->OutputBrowser->moveCursor(QTextCursor::End);
    return;
  }

  while (nh_.getParam("/mavros/whud_basic/ack_result", ack_result)) {
    if (ack_result == 0) {
      ui_->OutputBrowser->setTextColor(QColor("green"));
      ui_->OutputBrowser->insertPlainText("Action done!\n");
      ui_->OutputBrowser->moveCursor(QTextCursor::End);
      return;
    }
  }
}
