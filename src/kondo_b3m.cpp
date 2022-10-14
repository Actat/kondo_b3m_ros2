#include "kondo_b3m.hpp"

KondoB3m::KondoB3m() : Node("kondo_b3m") {
  using namespace std::chrono_literals;

  this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 1500000);
  this->declare_parameter<std::vector<std::string>>("motor_list", {});

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baudrate", baudrate_);

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
      "b3m_joint_state", rclcpp::QoS(10));
  if (motor_list_.size() > 0) {
    timer_ = this->create_wall_timer(
        20ms, std::bind(&KondoB3m::publishJointState, this));
  }
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
}

KondoB3m::~KondoB3m() {
  std::vector<uint8_t> id(1);
  std::vector<uint8_t> data(1);
  id[0]   = 255;
  data[0] = 0x02;
  port_->commandWrite(1, &id[0], 1, &data[0], 0x28);
}

// private---------------------------------------------------------------------

void KondoB3m::publishJointState() {
  const int READ_LEN = 8;
  std::vector<std::string> name;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<uint8_t> buf(READ_LEN * motor_list_.size());

  std::vector<uint8_t> id_list(motor_list_.size());
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
}

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
    B3mMotor motor                           = *(std::find_if(
                                  motor_list_.begin(), motor_list_.end(),
                                  [&pos](B3mMotor motor) { return motor.get_id() == pos.id; }));

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
    B3mMotor motor                        = *(std::find_if(
                               motor_list_.begin(), motor_list_.end(),
                               [&spd](B3mMotor motor) { return motor.get_id() == spd.id; }));

    double deg_s    = motor.get_direction_sign() * rad_s * 360 / 2 / M_PI;
    int16_t cmd     = (int16_t)(deg_s * 100);
    data[i * 2]     = (cmd & 0xFF);
    data[i * 2 + 1] = ((cmd >> 8) & 0xFF);
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 2, &data[0], 0x30);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KondoB3m>());
  rclcpp::shutdown();
  return 0;
}
