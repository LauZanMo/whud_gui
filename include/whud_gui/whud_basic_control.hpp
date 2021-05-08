#ifndef WHUD_BASIC_CONTROL_HPP
#define WHUD_BASIC_CONTROL_HPP

#include <ros/ros.h>

#include <iostream>

#include "ui_widget.h"

class whud_basic_control : public QObject {
  Q_OBJECT
 public:
  whud_basic_control(QObject* parent = nullptr, Ui::Widget* ui = nullptr);
  ~whud_basic_control();
  void ChangeComboBox(int index);
  void ClickSendButton();

 private:
  Ui::Widget* ui_;
  ros::NodeHandle nh_;

  QDoubleValidator* take_off_validator_1_;
  QDoubleValidator* take_off_validator_2_;
  QDoubleValidator* land_validator_1_;
  QDoubleValidator* set_height_validator_1_;
  QDoubleValidator* set_height_validator_2_;
  QDoubleValidator* set_yaw_validator_1_;
  QIntValidator* set_yaw_validator_2_;

  void SetNotation();

  void SetParamLabel(const char* label1, const char* label2);
  void ClearParamEditor();
  void SetParamValidator(QDoubleValidator* validator1,
                         QDoubleValidator* validator2);
  void SetParamValidator(QDoubleValidator* validator);
  void SetParamValidator(QDoubleValidator* validator1,
                         QIntValidator* validator2);

  bool ParamCheck(int num);
  void SendBasicCmd(int index);
};

#endif  // WHUD_BASIC_CONTROL_HPP
