#include "b3m_motor.hpp"

B3mMotor::B3mMotor(uint8_t id,
                   std::string name,
                   bool direction,
                   double offset) {
  initialize_(id, name, direction, offset);
}

B3mMotor::B3mMotor(std::string json_string) {
  size_t itr      = 0;
  std::string key = "";
  std::string val = "";

  uint8_t id;
  std::string name = "";
  bool direction   = true;
  double offset    = 0;

  // JSON should be like this:
  // {"id": 0, "name": "joint0", "direction": true, "offset": 0}
  while (itr < json_string.size()) {
    if (json_string.at(itr) == '{' || json_string.at(itr) == '}' ||
        json_string.at(itr) == ' ' || json_string.at(itr) == ',') {
      itr++;
    } else if (json_string.at(itr) == '"') {
      itr++;
      while (json_string.at(itr) != '"') {
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
        direction = (val == "true");
      } else if (key == "offset") {
        offset = std::stod(val);
      }
      key = "";
      val = "";
    }
  }

  initialize_(id, name, direction, offset);
}

void B3mMotor::initialize_(uint8_t id,
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
}
