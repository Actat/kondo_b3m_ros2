#include "b3m_port.cpp"
#include "kondo_b3m_interfaces/srv/desired_speed.hpp"
#include "kondo_b3m_interfaces/srv/motor_free.hpp"
#include "kondo_b3m_interfaces/srv/start_speed_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>

#include <iostream>

B3mPort *port;

void motorFree(
    const std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Request>
        request,
    std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Response> response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i] = request->id[i];
    data[i] = 0x02;
  }
  response->success = port->commandWrite(request->data_len, &id[0], 1,
                                         (uint8_t *)&data[0], 0x28);
}

void startSpeedControl(
    const std::shared_ptr<kondo_b3m_interfaces::srv::StartSpeedControl::Request>
        request,
    std::shared_ptr<kondo_b3m_interfaces::srv::StartSpeedControl::Response>
        response) {
  std::vector<uint8_t> id(request->data_len);
  std::vector<uint8_t> data(request->data_len);
  for (int i = 0; i < request->data_len; i++) {
    id[i] = request->id[i];
    data[i] = 0b00000100;
  }
  response->success = port->commandWrite(request->data_len, &id[0], 1,
                                         (uint8_t *)&data[0], 0x28);
}

void desiredSpeed(
    const std::shared_ptr<kondo_b3m_interfaces::srv::DesiredSpeed::Request>
        request,
    std::shared_ptr<kondo_b3m_interfaces::srv::DesiredSpeed::Response>
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
  response->success = port->commandWrite(request->data_len, &id[0], 2,
                                         (uint8_t *)&data[0], 0x30);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  port = new B3mPort("/dev/ttyUSB0", 1500000);

  uint8_t id[1] = {0x00};
  uint8_t data[1][1];

  data[0][0] = 0x02;
  std::cout << "free the motor." << std::endl;
  std::cout << port->commandWrite(1, id, 1, (uint8_t *)data, 0x28) << std::endl;

  data[0][0] = 0x00;
  std::cout << "set PID gain preset No. 0" << std::endl;
  std::cout << port->commandWrite(1, id, 1, (uint8_t *)data, 0x5C) << std::endl;

  data[0][0] = 0x00;
  std::cout << "start position control" << std::endl;
  std::cout << port->commandWrite(1, id, 1, (uint8_t *)data, 0x28) << std::endl;

  uint8_t dest[1][2];
  dest[0][0] = 0x00;
  dest[0][1] = 0x00;
  std::cout << "set dest to 0 deg." << std::endl;
  std::cout << port->commandWrite(1, id, 2, (uint8_t *)dest, 0x2A) << std::endl;
  sleep(2);

  dest[0][0] = 0x50;
  dest[0][1] = 0x46;
  std::cout << "set dest to 180 deg." << std::endl;
  std::cout << port->commandWrite(1, id, 2, (uint8_t *)dest, 0x2A) << std::endl;
  sleep(2);

  dest[0][0] = 0x00;
  dest[0][1] = 0x00;
  std::cout << "set dest to 0 deg." << std::endl;
  std::cout << port->commandWrite(1, id, 2, (uint8_t *)dest, 0x2A) << std::endl;
  sleep(2);

  /*
  data[0][0] = 0x02;
  std::cout << "free the motor." << std::endl;
  std::cout << port->commandWrite(1, id, 1, (uint8_t *)data, 0x28) << std::endl;
  */

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("kondo_b3m");
  rclcpp::Service<kondo_b3m_interfaces::srv::MotorFree>::SharedPtr
      service_free_motor =
          node->create_service<kondo_b3m_interfaces::srv::MotorFree>(
              "kondo_b3m_free_motor", &motorFree);
  rclcpp::Service<kondo_b3m_interfaces::srv::StartSpeedControl>::SharedPtr
      service_start_speed_control =
          node->create_service<kondo_b3m_interfaces::srv::StartSpeedControl>(
              "kondo_b3m_start_speed_control", &startSpeedControl);
  rclcpp::Service<kondo_b3m_interfaces::srv::DesiredSpeed>::SharedPtr
      service_desired_speed =
          node->create_service<kondo_b3m_interfaces::srv::DesiredSpeed>(
              "kondo_b3m_desired_speed", &desiredSpeed);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
