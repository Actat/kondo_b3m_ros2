#include <cstdio>
#include "b3m_port.cpp"

#include <iostream>

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  printf("hello world kondo_b3m_ros2 package\n");

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

  uint8_t id[1] = {0};
  port->reset(id, 0);

  return 0;
}
