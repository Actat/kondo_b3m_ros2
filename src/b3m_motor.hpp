#ifndef B3M_MOTOR_HPP_
#define B3M_MOTOR_HPP_

#include <string>

class B3mMotor {
public:
  B3mMotor(uint8_t id,
           std::string name = "",
           bool direction   = true,
           double offset    = 0) {
    motor_id_        = id;
    joint_direction_ = direction;
    joint_offset_    = offset;
    if (name.length() == 0) {
      joint_name_ = std::to_string(id);
    } else {
      joint_name_ = name;
    }
  };
  B3mMotor(std::string json_string);

  uint8_t get_id() { return motor_id_; };
  std::string get_name() { return joint_name_; };
  bool get_direction() { return joint_direction_; };
  double get_offset() { return joint_offset_; };

  int get_direction_sign() { return joint_direction_ ? 1 : -1; };

private:
  uint8_t motor_id_;
  std::string joint_name_;
  bool joint_direction_;
  double joint_offset_;
};

B3mMotor::B3mMotor(std::string json_string) {
  size_t itr      = 0;
  std::string key = "";
  std::string val = "";

  uint8_t id;
  std::string name = "";
  bool direction   = true;
  double offset    = 0;

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
      while (json_string.at(itr) == ' ' || json_string.at(itr) == ':') {
        itr++;
      }

      if (key == "id") {
        while (json_string.at(itr) != ' ') {
          val += json_string.at(itr);
          itr++;
        }
        id = std::stoi(val);
      } else if (key == "name") {
        itr++;
        while (json_string.at(itr) != '"' || json_string.at(itr - 1) == '\\') {
          val += json_string.at(itr);
          itr++;
        }
        itr++;
        name = val;
      } else if (key == "direction") {
        while (json_string.at(itr) != ' ') {
          val += json_string.at(itr);
          itr++;
        }
        direction = (val == "true");
      } else if (key == "offset") {
        while (json_string.at(itr) != ' ') {
          val += json_string.at(itr);
          itr++;
        }
        offset = std::stod(val);
      }
      key = "";
      val = "";
    }
  }

  motor_id_        = id;
  joint_direction_ = direction;
  joint_offset_    = offset;
  if (name.length() == 0) {
    joint_name_ = std::to_string(id);
  } else {
    joint_name_ = name;
  }
}

#endif  // B3M_MOTOR_HPP_
