#include "b3m_motor.hpp"

B3mMotor::B3mMotor(unsigned char id,
                   std::string name,
                   bool direction,
                   double offset) {
  initialize_(id, name, direction, offset);
}

B3mMotor::B3mMotor(std::string json_string) {
  size_t itr      = 0;
  std::string key = "";
  std::string val = "";

  unsigned char id;
  std::string name = "";
  bool direction   = true;
  double offset    = 0;

  // JSON should be like this:
  // {"id": 0, "name": "joint0", "direction": true, "offset": 0}
  while (itr < json_string.size()) {
    if (json_string.at(itr) == '{' || json_string.at(itr) == '}' ||
        json_string.at(itr) == ' ' || json_string.at(itr) == ',') {
      itr++;
    } else if (json_string.at(itr) == '"' || json_string.at(itr) == '\'') {
      itr++;
      while (json_string.at(itr) != '"' && json_string.at(itr) != '\'') {
        key += json_string.at(itr);
        itr++;
      }
      itr++;  // the '"' following 'key'
      while (json_string.at(itr) == ' ' || json_string.at(itr) == ':') {
        itr++;
      }

      while (json_string.at(itr) != ' ' && json_string.at(itr) != ',' &&
             json_string.at(itr) != '}') {
        val += json_string.at(itr);
        itr++;
      }
      if (key == "id") {
        id = std::stoi(val);
      } else if (key == "name") {
        name = val.substr(1, val.size() - 2);
      } else if (key == "direction") {
        direction = (val == "true" || val == "True");
      } else if (key == "offset") {
        offset = std::stod(val);
      }
      key = "";
      val = "";
    }
  }

  initialize_(id, name, direction, offset);
}

void B3mMotor::initialize_(unsigned char id,
                           std::string name,
                           bool direction,
                           double offset) {
  motor_id_        = id;
  joint_direction_ = direction;
  joint_offset_    = offset;
  if (name.length() == 0) {
    joint_name_ = std::to_string(id);
  } else {
    joint_name_ = name;
  }
  status_.fill(0);
}

void B3mMotor::set_status(size_t select, unsigned char status) {
  switch (select) {
    case 0:
      if (status == 0) {
        status_.fill(0);
        break;
      }
      // fall through
    case 1:
    case 2:
    case 3:
    case 4:
      status_.at(select) = status;
      break;

    case 7:
      status_.fill(0);
      break;

    default:
      break;
  }
}

unsigned char B3mMotor::get_option_byte() const {
  if (status_.at(0) == 0b000) {
    return 0b000;
  } else if ((status_.at(0) & 0b0001) == 0b0001 && status_.at(1) == 0) {
    // system status error
    return 0b001;
  } else if ((status_.at(0) & 0b0010) == 0b0010 && status_.at(2) == 0) {
    // motor status error
    return 0b010;
  } else if ((status_.at(0) & 0b0100) == 0b0100 && status_.at(3) == 0) {
    // uart status error
    return 0b011;
  } else if ((status_.at(0) & 0b1000) == 0b1000 && status_.at(4) == 0) {
    // command status error
    return 0b100;
  }

  return 0b10000000;
}
