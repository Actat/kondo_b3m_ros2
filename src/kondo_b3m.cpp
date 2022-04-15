#include "kondo_b3m.hpp"

KondoB3m::KondoB3m() : Node("kondo_b3m") {
  using namespace std::chrono_literals;

  port_ = new B3mPort("/dev/ttyUSB0", 1500000);
  publisher_ = this->create_publisher<std_msgs::msg::String>("B3m_joint_state",
                                                             rclcpp::QoS(10));
  timer_ = this->create_wall_timer(
      500ms, std::bind(&KondoB3m::publishJointState, this));
  service_free_motor_ =
      this->create_service<kondo_b3m_interfaces::srv::MotorFree>(
          "kondo_b3m_free_motor",
          std::bind(&KondoB3m::motorFree, this, std::placeholders::_1,
                    std::placeholders::_2));
  service_start_speed_control_ =
      this->create_service<kondo_b3m_interfaces::srv::StartSpeedControl>(
          "kondo_b3m_start_speed_control",
          std::bind(&KondoB3m::startSpeedControl, this, std::placeholders::_1,
                    std::placeholders::_2));
  service_desired_speed_ =
      this->create_service<kondo_b3m_interfaces::srv::DesiredSpeed>(
          "kondo_b3m_desired_speed",
          std::bind(&KondoB3m::desiredSpeed, this, std::placeholders::_1,
                    std::placeholders::_2));
}

// private---------------------------------------------------------------------
void KondoB3m::publishJointState() {
  auto message = std_msgs::msg::String();
  message.data = "test";
  publisher_->publish(message);
}

void KondoB3m::motorFree(
    const std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Request>
        request,
    const std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Response>
        response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i] = request->id[i];
    data[i] = 0x02;
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 1, &data[0], 0x28);
}

void KondoB3m::startSpeedControl(
    const std::shared_ptr<kondo_b3m_interfaces::srv::StartSpeedControl::Request>
        request,
    const std::shared_ptr<
        kondo_b3m_interfaces::srv::StartSpeedControl::Response>
        response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  std::vector<uint8_t> gain(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i] = request->id[i];
    data[i] = 0b00000100;
    gain[i] = 0x01;
  }
  response->success =
      port_->commandWrite(request->data_len, &id[0], 1, &data[0], 0x28) &&
      port_->commandWrite(request->data_len, &id[0], 1, &data[0], 0x5C);
}

void KondoB3m::desiredSpeed(
    const std::shared_ptr<kondo_b3m_interfaces::srv::DesiredSpeed::Request>
        request,
    const std::shared_ptr<kondo_b3m_interfaces::srv::DesiredSpeed::Response>
        response) {
  std::vector<kondo_b3m_interfaces::msg::DesiredSpeed> speed = request->speed;
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len * 2);
  for (int i = 0; i < request->data_len; i++) {
    kondo_b3m_interfaces::msg::DesiredSpeed spd = speed[i];
    id[i] = spd.id;
    int16_t cmd = (int16_t)(spd.speed * 100);
    data[i * 2] = (cmd & 0xFF);
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
