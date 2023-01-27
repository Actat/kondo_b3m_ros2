#ifndef B3M_PORT_HPP_
#define B3M_PORT_HPP_

#include <fcntl.h>
// #include <sys/ioctl.h>
// #include <sys/stat.h>
// #include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
// #include <chrono>
// #include <cmath>
// #include <map>
#include <rclcpp/rclcpp.hpp>
// #include <stdexcept>
// #include <string>
#include <vector>
#include "b3m_command.hpp"

// #define B3M_COMMAND_MAX_LENGTH 256

class B3mPort {
public:
  B3mPort(std::string device_name, uint32_t baudrate);
  ~B3mPort();
  bool wright_device(B3mCommand const &command);
  B3mCommand read_device();

private:
  uint32_t baudrate_;
  bool initialized_;
  std::string device_name_;
  int device_file_;
  timespec guard_time_;

  bool read_(unsigned char *buf, size_t nbytes);
  tcflag_t getCBAUD();
  timespec getGuardTime();
};

#endif  // B3M_PORT_HPP_
