#ifndef B3M_COMMAND_HPP_
#define B3M_COMMAND_HPP_

#include <array>
#include <vector>
#include "rclcpp/rclcpp.hpp"

unsigned char const B3M_COMMAND_LOAD           = 0x01;
unsigned char const B3M_COMMAND_SAVE           = 0x02;
unsigned char const B3M_COMMAND_READ           = 0x03;
unsigned char const B3M_COMMAND_WRITE          = 0x04;
unsigned char const B3M_COMMAND_RESET          = 0x05;
unsigned char const B3M_COMMAND_POSITION       = 0x06;
unsigned char const B3M_COMMAND_LOAD_REPLY     = 0x81;
unsigned char const B3M_COMMAND_SAVE_REPLY     = 0x82;
unsigned char const B3M_COMMAND_READ_REPLY     = 0x83;
unsigned char const B3M_COMMAND_WRITE_REPLY    = 0x84;
unsigned char const B3M_COMMAND_POSITION_REPLY = 0x86;

class B3mCommand {
public:
  B3mCommand();
  B3mCommand(std::vector<unsigned char> const &command);

  rclcpp::Time time() const { return created_time_; };
  std::vector<unsigned char> buf() const;  // Address of command byte sequence
  unsigned char size() const;              // Size of command byte sequence
  unsigned char command() const { return command_; };
  unsigned char option() const { return option_; };
  unsigned char id() const { return id_; };
  std::vector<unsigned char> data() const { return data_; };
  unsigned char sum() const;
  bool validated() const;
  bool expect_reply() const;
  void set_command(unsigned char command);
  void set_option(unsigned char option);
  void set_id(unsigned char id);
  void set_data(std::vector<unsigned char> data);
  void set_validated();

private:
  std::array<bool, 4> validated_;  // command, option, id, checksum
  rclcpp::Time created_time_;
  unsigned char command_;
  unsigned char option_;
  unsigned char id_;
  std::vector<unsigned char> data_;

  void initialize_();
};

#endif  // B3M_COMMAND_HPP_
