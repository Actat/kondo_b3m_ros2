#ifndef B3M_MOTOR_HPP_
#define B3M_MOTOR_HPP_

#include <array>
#include <string>
#include "b3m_command.hpp"

unsigned char const B3M_MOTOR_MODE_F = 0b00000010;
unsigned char const B3M_MOTOR_MODE_P = 0b00000000;
unsigned char const B3M_MOTOR_MODE_S = 0b00000100;
unsigned char const B3M_MOTOR_MODE_T = 0b00001000;

class B3mMotor {
public:
  B3mMotor(unsigned char id,
           std::string name = "",
           bool direction   = true,
           double offset    = 0);
  B3mMotor(std::string json_string);

  unsigned char id() const { return motor_id_; };
  std::string name() const { return joint_name_; };
  bool direction() const { return joint_direction_; };
  double offset() const { return joint_offset_; };
  int get_direction_sign() const { return joint_direction_ ? 1 : -1; };
  unsigned char control_mode() const { return control_mode_; };
  unsigned char get_status(size_t select) const { return status_.at(select); };
  void set_control_mode(unsigned char mode);
  void set_status(size_t select, unsigned char status);
  unsigned char get_option_byte() const;

private:
  unsigned char control_mode_;
  unsigned char motor_id_;
  std::string joint_name_;
  bool joint_direction_;
  double joint_offset_;
  std::array<unsigned char, 5> status_;

  void initialize_(unsigned char id,
                   std::string name = "",
                   bool direction   = true,
                   double offset    = 0);
};

#endif  // B3M_MOTOR_HPP_
