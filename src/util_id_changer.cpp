#include <iostream>
#include <vector>
#include "b3m_port.hpp"

int main() {
  std::cout << "Kondo B3M ID changer" << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate    = 1500000;
  B3mPort *port         = new B3mPort(port_name, baud_rate);

  std::cout << "Port is opened." << std::endl;
  std::cout << "Searching for motors. Please wait..." << std::endl;

  std::vector<uint8_t> motor_list;
  for (uint8_t i = 0; i < 0xFE; i++) {
    uint8_t buf;
    if (port->commandRead(i, 0x00, 1, &buf)) {
      motor_list.push_back(i);
    }
  }

  int n_motor = motor_list.size();
  if (n_motor == 0) {
    std::cout << "No motor is found." << std::endl;
    std::cout << "abort." << std::endl;
    return 1;
  }

  uint8_t id_from[1];
  if (n_motor == 1) {
    id_from[0] = motor_list[0];
    std::cout << "One motor is found (id: " << (int)id_from[0] << ")."
              << std::endl;
  } else {
    std::cout << "Multiple motors are found." << std::endl;
    std::cout << "[";
    for (auto id : motor_list) {
      std::cout << (int)id << ", ";
    }
    std::cout << "\b\b]" << std::endl;
    int input;
    std::cout << "Select the motor to change the id. Enter the id: ";
    std::cin >> input;
    id_from[0] = input;
  }

  uint8_t id_to[1];
  int input;
  std::cout << "Enter the ID to change to: ";
  std::cin >> input;
  if (input < 0 || input > 0xFE) {
    std::cout << "ID must be in the range 0 - 254." << std::endl;
    std::cout << "abort." << std::endl;
    return 2;
  }
  id_to[0] = input;

  std::cout << "The motor ID is changed from " << (int)id_from[0] << " to "
            << (int)id_to[0] << ". Are you sure? [y/n]: ";
  char buf;
  std::cin >> buf;
  if (buf != 'y') {
    std::cout << "abort." << std::endl;
    return 3;
  }

  if (!port->commandWrite(1, id_from, 1, id_to, 0x00)) {
    std::cout << "Write command failed." << std::endl;
    return 4;
  }

  if (!port->commandSave(1, id_to)) {
    std::cout << "Save command failed." << std::endl;
    return 5;
  }

  if (!port->commandReset(1, id_to)) {
    std::cout << "Reset command failed." << std::endl;
    return 6;
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
