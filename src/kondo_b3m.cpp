#include "b3m_port.cpp"
#include "kondo_b3m_interfaces/srv/motor_free.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>

B3mPort *port;

void motorFree(
    const std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Request>
        request,
    std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Response> response) {
  uint8_t id[request->data_len];
  uint8_t data[request->data_len][1];
  for (int i = 0; i < request->data_len; i++) {
    id[i] = request->id[i];
    data[i][0] = 0x02;
  }
  response->success =
      port->commandWrite(request->data_len, id, 1, (uint8_t *)data, 0x28);
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

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("kondo_b3m_free_motor");
  rclcpp::Service<kondo_b3m_interfaces::srv::MotorFree>::SharedPtr service =
      node->create_service<kondo_b3m_interfaces::srv::MotorFree>(
          "kondo_b3m_free_motor", &motorFree);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
