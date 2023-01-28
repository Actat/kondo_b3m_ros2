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
  /*
  service_desired_ = this->create_service<kondo_b3m_ros2::srv::Desired>(
      "~/desired", std::bind(&KondoB3m::desired_, this, std::placeholders::_1,
                             std::placeholders::_2));
  */

  for (auto &motor : motor_list_) {
    B3mCommand cmd(B3M_COMMAND_READ, motor.get_option_byte(), motor.id(),
                   std::vector<unsigned char>({0x28, 0x01}));
    auto reply = send_command_(cmd);
    motor.set_control_mode(reply.data().at(0));
  }

  /*
  service_free_motor_ = this->create_service<kondo_b3m_ros2::srv::MotorFree>(
      "kondo_b3m_free_motor",
      std::bind(&KondoB3m::motorFree, this, std::placeholders::_1,
                std::placeholders::_2));
  service_start_position_control_ =
      this->create_service<kondo_b3m_ros2::srv::StartPositionControl>(
          "kondo_b3m_start_position_control",
          std::bind(&KondoB3m::startPositionControl, this,
                    std::placeholders::_1, std::placeholders::_2));
  service_start_speed_control_ =
      this->create_service<kondo_b3m_ros2::srv::StartSpeedControl>(
          "kondo_b3m_start_speed_control",
          std::bind(&KondoB3m::startSpeedControl, this, std::placeholders::_1,
                    std::placeholders::_2));
  service_desired_position_ =
      this->create_service<kondo_b3m_ros2::srv::DesiredPosition>(
          "kondo_b3m_desired_position",
          std::bind(&KondoB3m::desiredPosition, this, std::placeholders::_1,
                    std::placeholders::_2));
  service_desired_speed_ =
      this->create_service<kondo_b3m_ros2::srv::DesiredSpeed>(
          "kondo_b3m_desired_speed",
          std::bind(&KondoB3m::desiredSpeed, this, std::placeholders::_1,
                    std::placeholders::_2));
  */
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

  /*
  const int READ_LEN = 8;
  std::vector<std::string> name;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<uint8_t> buf(READ_LEN * motor_list_.size());

  auto id_list = std::vector<uint8_t>();
  std::for_each(
      motor_list_.begin(), motor_list_.end(),
      [&id_list](B3mMotor motor) { id_list.push_back(motor.get_id()); });
  std::vector<bool> is_success = port_->commandMultiMotorRead(
      motor_list_.size(), &id_list[0], 0x2C, READ_LEN, &buf[0]);

  for (uint i = 0; i < motor_list_.size(); i++) {
    if (!is_success[i]) {
      continue;
    }
    int16_t p = buf[i * READ_LEN + 1] << 8 | buf[i * READ_LEN + 0];
    int16_t v = buf[i * READ_LEN + 7] << 8 | buf[i * READ_LEN + 6];
    double position =
        std::fmod(2.0 * M_PI * p / 100 / 360 - motor_list_[i].get_offset(),
                  2 * 3.14115926536);
    position = position > 3.1415926536    ? position - 2 * 3.1415926536
               : position < -3.1415926536 ? position + 2 * 3.1415926536
                                          : position;
    name.push_back(motor_list_[i].get_name());
    pos.push_back(motor_list_[i].get_direction_sign() * position);
    vel.push_back(motor_list_[i].get_direction_sign() * 2.0 * M_PI * v /
                  (100 * 360));
  }

  auto message            = sensor_msgs::msg::JointState();
  message.header          = std_msgs::msg::Header();
  message.header.stamp    = rclcpp::Clock().now();
  message.header.frame_id = port_name_;
  message.name            = name;
  message.position        = pos;
  message.velocity        = vel;

  publisher_->publish(message);
  */
}

void KondoB3m::control_mode_(
    std::shared_ptr<kondo_b3m_ros2::srv::ControlMode::Request> const request,
    std::shared_ptr<kondo_b3m_ros2::srv::ControlMode::Response> response) {
  if (request->name.size() != request->mode.size()) {
    response->success = false;
    return;
  }

  response->success = true;

  for (size_t i = 0; i < request->name.size(); ++i) {
    auto const name = request->name.at(i);
    auto const mode = request->mode.at(i);
    B3mMotor *motor;
    auto found =
        std::find_if(motor_list_.begin(), motor_list_.end(),
                     [&name](auto const elem) { return elem.name() == name; });
    if (found == motor_list_.end()) {
      RCLCPP_WARN(this->get_logger(),
                  "Joint '" + name + "' is not found in motor_list_");
      response->success = false;
      continue;
    }
    motor = &*found;

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
    if (motor->control_mode() == mode_byte) {
      continue;
    }

    B3mCommand cmd_free(B3M_COMMAND_WRITE, motor->get_option_byte(),
                        motor->id(),
                        std::vector<unsigned char>({0b00000010, 0x28, 0x01}));
    auto reply_free = send_command_(cmd_free);
    if (!reply_free.validated()) {
      RCLCPP_WARN(
          this->get_logger(),
          "Motor is not free due to write failure. Joint '" + name + "'");
      response->success = false;
      continue;
    }
    motor->set_control_mode(0b00000010);

    if (mode_byte == 0b00000010) {  // change to free
      continue;
    }

    B3mCommand cmd_gain(B3M_COMMAND_WRITE, motor->get_option_byte(),
                        motor->id(),
                        std::vector<unsigned char>({gain_byte, 0x5c, 0x01}));
    auto reply_gain = send_command_(cmd_gain);
    if (!reply_gain.validated()) {
      RCLCPP_WARN(this->get_logger(),
                  "Gain is not set due to write failure. Joint '" + name + "'");
      response->success = false;
      continue;
    }

    B3mCommand cmd_mode(B3M_COMMAND_WRITE, motor->get_option_byte(),
                        motor->id(),
                        std::vector<unsigned char>({mode_byte, 0x28, 0x01}));
    auto reply_mode = send_command_(cmd_mode);
    if (!reply_mode.validated()) {
      RCLCPP_WARN(this->get_logger(),
                  "Mode is not set due to write failure. Joint '" + name + "'");
      response->success = false;
      continue;
    }
    motor->set_control_mode(mode_byte);
  }
}

/*
void KondoB3m::desired_(
    std::shared_ptr<kondo_b3m_ros2::srv::Desired::Request> const request,
    std::shared_ptr<kondo_b3m_ros2::srv::Desired::Response> response) {}
*/

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

/*
void KondoB3m::motorFree(
    const std::shared_ptr<kondo_b3m_ros2::srv::MotorFree::Request> request,
    const std::shared_ptr<kondo_b3m_ros2::srv::MotorFree::Response> response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i]   = request->id[i];
    data[i] = 0x02;
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 1, &data[0], 0x28);
}

void KondoB3m::startPositionControl(
    const std::shared_ptr<kondo_b3m_ros2::srv::StartPositionControl::Request>
        request,
    const std::shared_ptr<kondo_b3m_ros2::srv::StartPositionControl::Response>
        response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  std::vector<uint8_t> gain(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i]   = request->id[i];
    data[i] = 0b00000000;
    gain[i] = 0x00;
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 1, &data[0], 0x28) &&
      port_->commandWrite(request->data_len, &id[0], 1, &gain[0], 0x5C);
}

void KondoB3m::startSpeedControl(
    const std::shared_ptr<kondo_b3m_ros2::srv::StartSpeedControl::Request>
        request,
    const std::shared_ptr<kondo_b3m_ros2::srv::StartSpeedControl::Response>
        response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  std::vector<uint8_t> gain(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i]   = request->id[i];
    data[i] = 0b00000100;
    gain[i] = 0x01;
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 1, &data[0], 0x28) &&
      port_->commandWrite(request->data_len, &id[0], 1, &gain[0], 0x5C);
}

void KondoB3m::desiredPosition(
    const std::shared_ptr<kondo_b3m_ros2::srv::DesiredPosition::Request>
        request,
    const std::shared_ptr<kondo_b3m_ros2::srv::DesiredPosition::Response>
        response) {
  std::vector<kondo_b3m_ros2::msg::DesiredPosition> position =
      request->position;
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len * 2);
  for (int i = 0; i < request->data_len; i++) {
    kondo_b3m_ros2::msg::DesiredPosition pos = position[i];
    id[i]                                    = pos.id;
    double rad                               = pos.position;
    B3mMotor motor                           = get_motor_(pos.id);

    double deg = motor.get_direction_sign() * (rad + motor.get_offset()) * 360 /
                 (2 * M_PI);
    int16_t cmd     = (int16_t)(deg * 100);
    data[i * 2]     = (cmd & 0xFF);
    data[i * 2 + 1] = ((cmd >> 8) & 0xFF);
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 2, &data[0], 0x2A);
}

void KondoB3m::desiredSpeed(
    const std::shared_ptr<kondo_b3m_ros2::srv::DesiredSpeed::Request> request,
    const std::shared_ptr<kondo_b3m_ros2::srv::DesiredSpeed::Response>
        response) {
  std::vector<kondo_b3m_ros2::msg::DesiredSpeed> speed = request->speed;
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len * 2);
  for (int i = 0; i < request->data_len; i++) {
    kondo_b3m_ros2::msg::DesiredSpeed spd = speed[i];
    id[i]                                 = spd.id;
    double rad_s                          = spd.speed;
    B3mMotor motor                        = get_motor_(spd.id);

    double deg_s    = motor.get_direction_sign() * rad_s * 360 / 2 / M_PI;
    int16_t cmd     = (int16_t)(deg_s * 100);
    data[i * 2]     = (cmd & 0xFF);
    data[i * 2 + 1] = ((cmd >> 8) & 0xFF);
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 2, &data[0], 0x30);
}

B3mMotor KondoB3m::get_motor_(uint8_t id) {
  auto func = [&id](B3mMotor motor) { return motor.get_id() == id; };
  B3mMotor motor =
      *(std::find_if(motor_list_.begin(), motor_list_.end(), func));
  return motor;
}
*/

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KondoB3m>());
  rclcpp::shutdown();
  return 0;
}
