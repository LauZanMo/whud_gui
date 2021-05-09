#include "whud_basic_control.hpp"

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

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

  conversion_pub_ =
      nh_.advertise<std_msgs::Bool>("/mavros/whud_nav/conversion", 1);
  take_off_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
      "/mavros/whud_basic/takeoff_height", 1);
  land_pub_ = nh_.advertise<std_msgs::Float64>("/mavros/whud_basic/land", 1);
  set_height_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
      "/mavros/whud_basic/height", 1);
  set_yaw_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/mavros/whud_basic/yaw", 1);

  QTimer* duty_timer = new QTimer(this);
  connect(duty_timer, &QTimer::timeout, this, &whud_basic_control::DutyLoop);
  duty_timer->start(100);
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
  if (SendBasicCmd(ui_->BasicCmdComboBox->currentIndex()) != 1)
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

uint8_t whud_basic_control::SendBasicCmd(int index) {
  uint8_t send_flag = 0;
  switch (index) {
    // take off
    case 1:
      if (!ParamCheck(2))
        break;
      else {
        take_off_height_ = ui_->Param1Editor->text().toDouble();
        take_off_speed_ = ui_->Param2Editor->text().toDouble();
        send_flag++;
        break;
      }
    // land
    case 2:
      if (!ParamCheck(1))
        break;
      else {
        land_speed_ = ui_->Param1Editor->text().toDouble();
        send_flag++;
        break;
      }
    // set height
    case 3:
      if (!ParamCheck(2))
        break;
      else {
        set_relative_height_ = ui_->Param1Editor->text().toDouble();
        set_height_speed_ = ui_->Param2Editor->text().toDouble();
        send_flag++;
        break;
      }
    // set yaw
    case 4:
      if (!ParamCheck(2))
        break;
      else {
        set_yaw_angle_ = ui_->Param1Editor->text().toDouble();
        set_yaw_type_ = ui_->Param2Editor->text().toDouble();
        send_flag++;
        break;
      }
    default:
      send_flag = 2;
      break;
  }

  if (send_flag == 0) {
    ui_->OutputBrowser->setTextColor(QColor("red"));
    ui_->OutputBrowser->insertPlainText("Please complete all parameters!\n");
    ui_->OutputBrowser->moveCursor(QTextCursor::End);
  } else if (send_flag == 1) {
    send_index_ = index;
    send_enable_ = true;
  } else {
    ui_->OutputBrowser->setTextColor(QColor("red"));
    ui_->OutputBrowser->insertPlainText("Please choose command!\n");
    ui_->OutputBrowser->moveCursor(QTextCursor::End);
  }

  return send_flag;
}

void whud_basic_control::DutyLoop() {
  // publish conversion msg
  if (ui_->ConversionCheckBox->checkState()) {
    std_msgs::Bool msg;
    msg.data = true;
    conversion_pub_.publish(msg);
  }

  // publish basic command msg
  if (send_enable_) {
    std_msgs::Float64MultiArray msg1;
    std_msgs::Float64 msg2;

    switch (send_index_) {
      // take off
      case 1:
        msg1.data = {take_off_speed_, take_off_height_};
        take_off_pub_.publish(msg1);
        break;
      // land
      case 2:
        msg2.data = land_speed_;
        land_pub_.publish(msg2);
        break;
      // set height
      case 3:
        msg1.data = {set_height_speed_, set_relative_height_};
        set_height_pub_.publish(msg1);
        break;
      // set yaw
      case 4:
        msg1.data = {set_yaw_angle_, set_yaw_type_};
        set_yaw_pub_.publish(msg1);
        break;
      default:
        break;
    }

    // send 5 times and reset
    if (++send_counter_ > 5) {
      send_enable_ = false;
      send_counter_ = 0;
      send_done_ = true;
    }
  }

  // wait progress ack
  if (send_done_) {
    int ack_index = -1;
    int ack_result = -1;

    nh_.getParam("/mavros/whud_basic/ack_cmd_index", ack_index);
    nh_.getParam("/mavros/whud_basic/ack_result", ack_result);

    if (++wait_counter_ <= 30) {
      // output browser update
      if (ack_index == send_index_ && ack_result == 5) {
        ui_->OutputBrowser->setTextColor(QColor("green"));
        ui_->OutputBrowser->insertPlainText("Send messages OK!\n");
        ui_->OutputBrowser->moveCursor(QTextCursor::End);

        // reset
        ack_done_ = true;
        send_done_ = false;
        wait_counter_ = 0;
      }
    } else {
      // output browser update
      ui_->OutputBrowser->setTextColor(QColor("red"));
      ui_->OutputBrowser->insertPlainText(
          "No ack from ACFLY, Maybe it is disconncted!\n");
      ui_->OutputBrowser->moveCursor(QTextCursor::End);

      // reset
      send_done_ = false;
      wait_counter_ = 0;
      ui_->SendButton->setEnabled(true);
    }
  }

  // wait action done
  if (ack_done_) {
    int ack_index = -1;
    int ack_result = -1;

    nh_.getParam("/mavros/whud_basic/ack_cmd_index", ack_index);
    nh_.getParam("/mavros/whud_basic/ack_result", ack_result);

    if (ack_index == send_index_ && ack_result == 0) {
      // output browser update
      ui_->OutputBrowser->setTextColor(QColor("green"));
      ui_->OutputBrowser->insertPlainText("Action done!\n");
      ui_->OutputBrowser->moveCursor(QTextCursor::End);

      // reset
      ack_done_ = false;
      ui_->SendButton->setEnabled(true);
    }
  }
}
