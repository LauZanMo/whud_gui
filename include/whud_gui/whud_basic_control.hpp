#ifndef WHUD_BASIC_CONTROL_HPP
#define WHUD_BASIC_CONTROL_HPP

#include <ros/ros.h>

#include <QTimer>

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

  ros::Publisher conversion_pub_;
  ros::Publisher take_off_pub_;
  ros::Publisher land_pub_;
  ros::Publisher set_height_pub_;
  ros::Publisher set_yaw_pub_;

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

  void DutyLoop();
};

#endif  // WHUD_BASIC_CONTROL_HPP
