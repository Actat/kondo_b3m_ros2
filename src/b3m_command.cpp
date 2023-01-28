#include "b3m_command.hpp"

B3mCommand::B3mCommand() {
  initialize_();
}

B3mCommand::B3mCommand(std::vector<unsigned char> const &command) {
  initialize_();

  unsigned char sum = 0;
  for (auto b : command) {
    sum += b;
  }
  if (sum != (2 * command.at(command.size() - 1)) % 256) {
    validated_.at(3) = false;
    RCLCPP_WARN(rclcpp::get_logger("B3mCommand"), "Invalid checksum.");
  }

  int command_length = command.at(0);
  set_command(command.at(1));
  set_option(command.at(2));
  set_id(command.at(3));
  auto vec = std::vector<unsigned char>();
  for (int i = 4; i < command_length - 1; ++i) {
    vec.push_back(command.at(i));
  }
  set_data(vec);
}

std::vector<unsigned char> B3mCommand::buf() const {
  std::vector<unsigned char> vec = {};
  vec.push_back(size());
  vec.push_back(command_);
  vec.push_back(option_);
  vec.push_back(id_);
  for (auto const b : data_) {
    vec.push_back(b);
  }
  unsigned char sum = 0;
  for (auto const b : vec) {
    sum += b;
  }
  vec.push_back(sum);

  return vec;
}

unsigned char B3mCommand::size() const {
  return 5 + data_.size();
}

bool B3mCommand::validated() const {
  return validated_.at(0) && validated_.at(1) &&  //
         validated_.at(2) && validated_.at(3);
}

bool B3mCommand::expect_reply() const {
  if (id_ == 255) {
    return false;
  }
  switch (command_) {
    case B3M_COMMAND_READ:
      return true;
    case B3M_COMMAND_RESET:
      return false;
    case B3M_COMMAND_LOAD:
    case B3M_COMMAND_SAVE:
      if (data_.size() > 0) {
        return false;
      }
      return true;
    case B3M_COMMAND_WRITE:
      if (data_.at(data_.size() - 1) == 1) {
        return true;
      }
      return false;
    case B3M_COMMAND_POSITION:
      if (data_.size() == 4) {
        return true;
      }
      return false;

    default:
      return false;
      break;
  }
}

void B3mCommand::set_command(unsigned char command) {
  if (command == B3M_COMMAND_LOAD || command == B3M_COMMAND_SAVE ||
      command == B3M_COMMAND_READ || command == B3M_COMMAND_WRITE ||
      command == B3M_COMMAND_RESET || command == B3M_COMMAND_POSITION ||
      command == B3M_COMMAND_LOAD_REPLY || command == B3M_COMMAND_SAVE_REPLY ||
      command == B3M_COMMAND_READ_REPLY || command == B3M_COMMAND_WRITE_REPLY ||
      command == B3M_COMMAND_POSITION_REPLY) {
    validated_.at(0) = true;
    command_         = command;
  } else {
    validated_.at(0) = false;
    command_         = 0;
  }
}

void B3mCommand::set_option(unsigned char option) {
  unsigned char select = (option & 0b01111111);
  if (select == 0b000 || select == 0b001 || select == 0b010 ||
      select == 0b011 || select == 0b100) {
    validated_.at(1) = true;
    option_          = option;
  } else {
    validated_.at(1) = false;
    option_          = 0;
  }
}

void B3mCommand::set_id(unsigned char id) {
  validated_.at(2) = true;
  id_              = id;
}

void B3mCommand::set_data(std::vector<unsigned char> data) {
  data_ = data;
}

void B3mCommand::set_validated() {
  validated_.fill(true);
}

void B3mCommand::initialize_() {
  created_time_ = rclcpp::Clock().now();

  validated_ = {false, false, false, true};
  command_   = 0;
  option_    = 0;
  id_        = 254;
  data_.clear();
}