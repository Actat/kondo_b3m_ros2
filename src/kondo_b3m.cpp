#include "b3m_port.cpp"

#include <iostream>

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  B3mPort *port = new B3mPort("/dev/ttyUSB0", 1500000);
  /*
  uint8_t data[4] = {
      (uint8_t)'d',
      (uint8_t)'a',
      (uint8_t)'t',
      (uint8_t)'a',
  };
  std::cout << port->writePort(data, 4) << std::endl;
  std::cout << data << std::endl;
  std::cout << port->readPort(data, 4) << std::endl;
  std::cout << data << std::endl;
  */

  uint8_t id[1] = {0x00};
  uint8_t data[1][1];

  data[0][0] = 0x02;
  std::cout << "free the motor." << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)data, 1, 0x28) << std::endl;

  data[0][0] = 0x00;
  std::cout << "set PID gain preset No. 0" << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)data, 1, 0x5C) << std::endl;

  data[0][0] = 0x00;
  std::cout << "start position control" << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)data, 1, 0x28) << std::endl;

  uint8_t dest[1][2];
  dest[0][0] = 0x00;
  dest[0][1] = 0x00;
  std::cout << "set dest to 0 deg." << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)dest, 2, 0x2A) << std::endl;
  sleep(2);

  dest[0][0] = 0x50;
  dest[0][1] = 0x46;
  std::cout << "set dest to 180 deg." << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)dest, 2, 0x2A) << std::endl;
  sleep(2);

  dest[0][0] = 0x00;
  dest[0][1] = 0x00;
  std::cout << "set dest to 0 deg." << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)dest, 2, 0x2A) << std::endl;
  sleep(2);

  data[0][0] = 0x02;
  std::cout << "free the motor." << std::endl;
  std::cout << port->commandWrite(id, 1, (uint8_t *)data, 1, 0x28) << std::endl;

  return 0;
}
