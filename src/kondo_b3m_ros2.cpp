#include <cstdio>
#include "b3m_port.cpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  printf("hello world kondo_b3m_ros2 package\n");

  B3mPort *port = new B3mPort("/dev/ttyUSB0");
  port->writePort("data", 4);
  char *data = "xxxx";
  port->readPort(data, 4);
  printf(data);

  return 0;
}
