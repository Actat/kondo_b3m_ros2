#include "../include/kondo_b3m_ros2/b3m_motor.hpp"

B3mMotor::B3mMotor(std::string json_string)
{
  size_t itr = 0;
  std::string key = "";
  std::string val = "";

  unsigned char id;
  std::string name = "";
  std::string mode = "";

  // JSON should be like this:
  // {"id": 0, "name": "joint", "mode": "position"}
  while (itr < json_string.size()) {
    if (json_string.at(itr) == '{' || json_string.at(itr) == '}' ||
      json_string.at(itr) == ' ' || json_string.at(itr) == ',')
    {
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
        json_string.at(itr) != '}')
      {
        val += json_string.at(itr);
        itr++;
      }
      if (key == "id") {
        id = std::stoi(val);
      } else if (key == "name") {
        name = val.substr(1, val.size() - 2);
      } else if (key == "mode") {
        mode = val.substr(1, val.size() - 2);
      }
      key = "";
      val = "";
    }
  }

  initialize_(id, name, mode);
}

void B3mMotor::initialize_(unsigned char id, std::string name, std::string mode)
{
  motor_id_ = id;
  joint_name_ = name;
  control_mode_ = mode;
}
