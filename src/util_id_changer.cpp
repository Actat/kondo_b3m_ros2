#include <iostream>
#include <vector>
#include "b3m_port.hpp"

int main(int argc, char **argv) {
  std::cout << "Kondo B3M ID changer" << std::endl;

  auto port_name = "/dev/ttyUSB0";
  auto baud_rate = 1500000;
  B3mPort *port  = new B3mPort(port_name, baud_rate);

  std::cout << "port opened." << std::endl;

  std::vector<uint8_t> motor_list;
  for (uint8_t i = 0; i < 0xFE; i++) {
    uint8_t buf;
    if (port->commandRead(i, 0x00, 1, &buf)) {
      motor_list.push_back(i);
    }
  }

  std::size_t n_motor = motor_list.size();
  if (n_motor == 0) {
    std::cout << "No motor is found." << std::endl;
    std::cout << "abort." << std::endl;
    return 1;
  }

  uint8_t *id_from = {0};
  if (n_motor == 1) {
    id_from[0] = motor_list[0];
    std::cout << "One motor is found (id: " << id_from[0] << ")." << std::endl;
  } else {
    std::cout << "Multiple motors are found." << std::endl;
    std::cout << "[";
    for (auto id : motor_list) {
      std::cout << id << ", ";
    }
    std::cout << "\b\b]" << std::endl;
    std::cout << "Select the motor to change the id. Enter the id: ";
    std::cin >> id_from[0];
  }

  uint8_t *id_to = {0};
  std::cout << "Enter the changed ID: ";
  std::cin >> id_to[0];

  std::cout << "The motor ID is changed from 0 to 1. Are you sure? [y/N]: ";
  std::string buf;
  std::cin >> buf;
  if (buf != "y") {
    std::cout << "abort." << std::endl;
    return 2;
  }

  if (!port->commandWrite(1, id_from, 1, id_to, 0x00)) {
    std::cout << "Write command failed." << std::endl;
    return 3;
  }

  if (!port->commandSave(1, id_to)) {
    std::cout << "Save command failed." << std::endl;
    return 4;
  }

  if (!port->commandReset(1, id_to)) {
    std::cout << "Reset command failed." << std::endl;
    return 5;
  }

  return 0;
}
