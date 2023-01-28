#include "kondo_b3m.hpp"

KondoB3m::KondoB3m() : Node("kondo_b3m") {
  this->declare_parameter<std::string>("port_name", "/dev/ttyKONDO");
  this->declare_parameter<int>("baudrate", 1500000);
  this->declare_parameter<int>("publish_frequency", 50);
  this->declare_parameter<std::vector<std::string>>("motor_list", {});

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("publish_frequency", publish_frequency_);

  motor_list_ = std::vector<B3mMotor>{};
  std::vector<std::string> motor_string_list;
  this->get_parameter("motor_list", motor_string_list);
  if (motor_string_list.size() > 0) {
    std::for_each(motor_string_list.begin(), motor_string_list.end(),
                  [this](std::string motor_string) {
                    motor_list_.push_back(B3mMotor(motor_string));
                  });
  }

  port_ = new B3mPort(port_name_, baudrate_);

  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_states", rclcpp::QoS(10));
  if (motor_list_.size() > 0) {
    timer_ = this->create_wall_timer(
        std::chrono::nanoseconds((int)1000000000.0 / publish_frequency_),
        std::bind(&KondoB3m::publishJointState, this));
  }

  service_control_mode_ =
      this->create_service<kondo_b3m_ros2::srv::ControlMode>(
          "~/control_mode",
          std::bind(&KondoB3m::control_mode_, this, std::placeholders::_1,
                    std::placeholders::_2));
  service_desired_ = this->create_service<kondo_b3m_ros2::srv::Desired>(
      "~/desired", std::bind(&KondoB3m::desired_, this, std::placeholders::_1,
                             std::placeholders::_2));

  for (auto &motor : motor_list_) {
    B3mCommand cmd(B3M_COMMAND_READ, motor.get_option_byte(), motor.id(),
                   std::vector<unsigned char>({0x28, 0x01}));
    auto reply = send_command_(cmd);
    motor.set_control_mode(reply.data().at(0));
  }
}

KondoB3m::~KondoB3m() {
  B3mCommand command(B3M_COMMAND_WRITE, 0x07, 255,
                     std::vector<unsigned char>({0x02, 0x28, 0x01}));
  send_command_(command);
}

// private---------------------------------------------------------------------

void KondoB3m::publishJointState() {
  for (auto const &motor : motor_list_) {
    B3mCommand command(B3M_COMMAND_READ, motor.get_option_byte(), motor.id(),
                       std::vector<unsigned char>({0x2C, 0x12}));
    auto reply = this->send_command_(command);
    if (!reply.validated()) {
      continue;
    }

    int16_t p = reply.data().at(1) << 8 | reply.data().at(0);
    int16_t v = reply.data().at(7) << 8 | reply.data().at(6);
    int16_t e = reply.data().at(17) << 8 | reply.data().at(16);

    auto message            = sensor_msgs::msg::JointState();
    message.header.stamp    = reply.time();
    message.header.frame_id = port_name_;
    message.name.push_back(motor.name());
    message.position.push_back(
        motor.get_direction_sign() *
        (std::fmod(M_PI * p / 18000 - motor.offset() + M_PI, 2 * M_PI) - M_PI));
    message.velocity.push_back(motor.get_direction_sign() * M_PI * v / 18000);
    message.effort.push_back(motor.get_direction_sign() * e / 1000.0);

    publisher_->publish(message);
  }
}

void KondoB3m::control_mode_(
    std::shared_ptr<kondo_b3m_ros2::srv::ControlMode::Request> const request,
    std::shared_ptr<kondo_b3m_ros2::srv::ControlMode::Response> response) {
  if (request->name.size() != request->mode.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Control mode is not set due to invalid request.'");
    response->success = false;
    return;
  }

  response->success = true;

  for (size_t i = 0; i < request->name.size(); ++i) {
    auto const mode = request->mode.at(i);
    unsigned char mode_byte, gain_byte;  // mode: 0x28, gain: 0x5c
    if (mode == "free" || mode == "fre" || mode == "f") {
      mode_byte = 0b00000010;
    } else if (mode == "position" || mode == "pos" || mode == "p") {
      mode_byte = 0b00000000;
      gain_byte = 0;
    } else if (mode == "speed" || mode == "spd" || mode == "s") {
      mode_byte = 0b00000100;
      gain_byte = 1;
    } else if (mode == "torque" || mode == "trq" || mode == "t") {
      mode_byte = 0b00001000;
      gain_byte = 2;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Control mode '" + mode + "' is not recognized.");
      continue;
    }

    auto const name = request->name.at(i);
    std::vector<B3mMotor>::iterator motor;
    if (name != "BROADCAST") {
      motor = std::find_if(
          motor_list_.begin(), motor_list_.end(),
          [&name](auto const elem) { return elem.name() == name; });
      if (motor == motor_list_.end()) {
        RCLCPP_WARN(this->get_logger(),
                    "Joint '" + name + "' is not found in motor_list_");
        response->success = false;
        continue;
      }
      if (motor->control_mode() == mode_byte) {
        continue;
      }
    }

    B3mCommand cmd_free(B3M_COMMAND_WRITE,
                        name == "BROADCAST" ? 0 : motor->get_option_byte(),
                        name == "BROADCAST" ? 255 : motor->id(),
                        std::vector<unsigned char>({0b00000010, 0x28, 0x01}));
    auto reply_free = send_command_(cmd_free);
    if (!reply_free.validated()) {
      RCLCPP_WARN(
          this->get_logger(),
          "Motor is not free due to write failure. Joint '" + name + "'");
      response->success = false;
      continue;
    }
    if (name == "BROADCAST") {
      for (auto &m : motor_list_) {
        m.set_control_mode(0b00000010);
      }
    } else {
      motor->set_control_mode(0b00000010);
    }

    if (mode_byte == 0b00000010) {  // change to free
      continue;
    }

    B3mCommand cmd_gain(B3M_COMMAND_WRITE,
                        name == "BROADCAST" ? 0 : motor->get_option_byte(),
                        name == "BROADCAST" ? 255 : motor->id(),
                        std::vector<unsigned char>({gain_byte, 0x5c, 0x01}));
    auto reply_gain = send_command_(cmd_gain);
    if (!reply_gain.validated()) {
      RCLCPP_WARN(this->get_logger(),
                  "Gain is not set due to write failure. Joint '" + name + "'");
      response->success = false;
      continue;
    }

    B3mCommand cmd_mode(B3M_COMMAND_WRITE,
                        name == "BROADCAST" ? 0 : motor->get_option_byte(),
                        name == "BROADCAST" ? 255 : motor->id(),
                        std::vector<unsigned char>({mode_byte, 0x28, 0x01}));
    auto reply_mode = send_command_(cmd_mode);
    if (!reply_mode.validated()) {
      RCLCPP_WARN(this->get_logger(),
                  "Mode is not set due to write failure. Joint '" + name + "'");
      response->success = false;
      continue;
    }
    if (name == "BROADCAST") {
      for (auto &m : motor_list_) {
        m.set_control_mode(mode_byte);
      }
    } else {
      motor->set_control_mode(mode_byte);
    }
  }
}

void KondoB3m::desired_(
    std::shared_ptr<kondo_b3m_ros2::srv::Desired::Request> const request,
    std::shared_ptr<kondo_b3m_ros2::srv::Desired::Response> response) {
  if (request->name.size() != request->value.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Desired value is not set due to invalid request.'");
    response->success = false;
    return;
  }

  response->success = true;

  for (size_t i = 0; i < request->name.size(); ++i) {
    auto const name = request->name.at(i);
    auto const motor =
        std::find_if(motor_list_.begin(), motor_list_.end(),
                     [&name](auto const elem) { return elem.name() == name; });
    if (motor == motor_list_.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "Joint '" + name + "' is not found in motor_list_");
      response->success = false;
      continue;
    }

    std::vector<unsigned char> data;
    if ((motor->control_mode() & 0b00001111) == 0b00000000) {  // position
      double deg = motor->get_direction_sign() *
                   (request->value.at(i) + motor->offset()) * 360 / (2 * M_PI);
      int16_t cmd = (int16_t)(deg * 100);
      data.push_back(cmd & 0xFF);
      data.push_back((cmd >> 8) & 0xFF);
      data.push_back(0x2A);
      data.push_back(0x01);
    } else if ((motor->control_mode() & 0b00001111) == 0b00000100) {  // speed
      double deg_s =
          motor->get_direction_sign() * request->value.at(i) * 360 / 2 / M_PI;
      int16_t cmd = (int16_t)(deg_s * 100);
      data.push_back(cmd & 0xFF);
      data.push_back((cmd >> 8) & 0xFF);
      data.push_back(0x30);
      data.push_back(0x01);
    } else if ((motor->control_mode() & 0b00001111) == 0b00001000) {  // torque
      double Nm   = motor->get_direction_sign() * request->value.at(i);
      int16_t cmd = (int16_t)(Nm * 1000);
      data.push_back(cmd & 0xFF);
      data.push_back((cmd >> 8) & 0xFF);
      data.push_back(0x3C);
      data.push_back(0x01);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Can not set desired value. Joint name: " + name);
      response->success = false;
      continue;
    }

    B3mCommand cmd(B3M_COMMAND_WRITE, motor->get_option_byte(), motor->id(),
                   data);
    auto reply = send_command_(cmd);
    if (!reply.validated()) {
      RCLCPP_WARN(this->get_logger(),
                  "Desired value is not set due to write failure. Joint '" +
                      name + "'");
      response->success = false;
      continue;
    }
  }
}

B3mCommand KondoB3m::send_command_(B3mCommand const &command) {
  if (!port_->wright_device(command)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send command.");
    return B3mCommand();
  }
  if (!command.expect_reply()) {
    B3mCommand cmd;
    cmd.set_validated();
    return cmd;
  }

  auto read = port_->read_device();
  if (!read.validated()) {
    RCLCPP_WARN(this->get_logger(), "Failed to read command.");
    return read;
  }

  auto itr = std::find_if(
      motor_list_.begin(), motor_list_.end(),
      [&command](auto const &elem) { return elem.id() == command.id(); });
  if (itr == motor_list_.end()) {
    return read;
  }

  itr->set_status(command.option(), read.option());
  return read;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KondoB3m>());
  rclcpp::shutdown();
  return 0;
}
