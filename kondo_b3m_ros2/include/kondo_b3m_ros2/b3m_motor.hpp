#ifndef B3M_MOTOR_HPP_
#define B3M_MOTOR_HPP_

#include <array>
#include <string>
#include "b3m_command.hpp"
#include "rclcpp/rclcpp.hpp"

unsigned char const B3M_MOTOR_MODE_F = 0b00000010;
unsigned char const B3M_MOTOR_MODE_P = 0b00000000;
unsigned char const B3M_MOTOR_MODE_S = 0b00000100;
unsigned char const B3M_MOTOR_MODE_T = 0b00001000;

class B3mMotor
{
public:
  B3mMotor(std::string json_string);

  unsigned char id() const {return motor_id_;}
  std::string name() const {return joint_name_;}
  std::string control_mode() const {return control_mode_;}

private:
  std::string control_mode_;
  unsigned char motor_id_;
  std::string joint_name_;

  void initialize_(
    unsigned char id,
    std::string name,
    std::string mode
  );
};

#endif  // B3M_MOTOR_HPP_
