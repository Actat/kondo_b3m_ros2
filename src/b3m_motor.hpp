#ifndef B3M_MOTOR_HPP_
#define B3M_MOTOR_HPP_

#include <array>
#include <string>
#include "b3m_command.hpp"

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
  unsigned char get_status(size_t select) const { return status_.at(select); };
  void set_status(size_t select, unsigned char status);
  unsigned char get_option_byte() const;

private:
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
