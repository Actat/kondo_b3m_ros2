#include "b3m_port.hpp"

B3mPort::B3mPort(std::string device_name, uint32_t baudrate) {
  initialized_ = false;
  baudrate_    = baudrate;
  device_name_ = device_name;
  guard_time_  = getGuardTime();

  tcflag_t baud = getCBAUD();
  if (baud == B0) {
    throw std::runtime_error("invalid baudrate");
  }

  RCLCPP_INFO(rclcpp::get_logger("B3mPort"),
              "Open device file: " + device_name_);
  device_file_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (device_file_ < 0) {
    throw std::runtime_error("Could not open device file: " + device_name_ +
                             ": " + std::to_string(device_file_));
  }

  tcflush(device_file_, TCIOFLUSH);

  struct termios tio;
  tio.c_iflag     = IGNPAR;
  tio.c_oflag     = 0;
  tio.c_cflag     = baud | CLOCAL | CREAD | CSTOPB;
  tio.c_lflag     = 0;
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;
  tcsetattr(device_file_, TCSANOW, &tio);

  initialized_ = true;
  return;
}

B3mPort::~B3mPort() {
  if (initialized_) {
    close(device_file_);
    initialized_ = false;
  }
}

bool B3mPort::wright_device(B3mCommand const &command) {
  if (!initialized_) {
    return false;
  }

  fd_set set;
  FD_ZERO(&set);
  FD_SET(device_file_, &set);
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 5 * 1000;
  int s           = select(device_file_ + 1, NULL, &set, NULL, &timeout);
  if (s == 0) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Failed to write due to timeout. (" + device_name_ + ")");
    return false;
  } else if (s == -1) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Error in the write_device function. (select errorno: %d)",
                errno);
    return false;
  }

  timespec rem = guard_time_;
  ssize_t size = write(device_file_, command.buf().data(), command.size());
  while (nanosleep(&rem, &rem) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Error in the write_device function. (nanosleep errorno: %d)",
                errno);
  }
  if (size != command.size()) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Error in the write_device function. (write errorno: %d)",
                errno);
    return false;
  }

  return true;
}

B3mCommand B3mPort::read_device() {
  if (!initialized_) {
    return B3mCommand();
  }

  unsigned char buf[B3M_COMMAND_MAX_LENGTH];
  try {
    read_(buf, 1);
  } catch (std::exception &e) {
    RCLCPP_WARN(
        rclcpp::get_logger("B3mPort"),
        "Failed to read size of command. Reason: " + (std::string)e.what());
    return B3mCommand();
  }
  try {
    read_(buf + 1, buf[0] - 1);
  } catch (std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Failed to read command. Reason: " + (std::string)e.what());
    return B3mCommand();
  }

  timespec rem = guard_time_;
  while (nanosleep(&rem, &rem) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Error in the read_device function. (nanosleep errorno: %d)",
                errno);
  }

  std::vector<unsigned char> vec(buf, buf + buf[0]);
  return B3mCommand(vec);
}

void B3mPort::read_(unsigned char *buf, size_t nbytes) {
  size_t rbytes = 0;
  while (rbytes < nbytes) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(device_file_, &set);
    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = 5 * 1000;
    int s           = select(device_file_ + 1, &set, NULL, NULL, &timeout);
    if (s == 0) {
      throw std::runtime_error("read_ failed due to timeout.");
    } else if (s == -1) {
      throw std::runtime_error("read_ failed. (select errorno: %d)");
    }

    ssize_t r = read(device_file_, buf + rbytes, nbytes);
    if (r > 0) {
      rbytes += r;
    } else if (r == 0) {
      throw std::runtime_error("Failed to read due to EOF.");
    } else {
      std::string error_msg = "Error in the B3mPort::read_. read error: ";
      if (errno == EAGAIN) {
        throw std::runtime_error(error_msg + "EAGAIN");
      } else if (errno == EWOULDBLOCK) {
        throw std::runtime_error(error_msg + "EWOULDBLOCK");
      } else if (errno == EBADF) {
        throw std::runtime_error(error_msg + "EBADF");
      } else if (errno == EFAULT) {
        throw std::runtime_error(error_msg + "EFAULT");
      } else if (errno == EINTR) {
        throw std::runtime_error(error_msg + "EINTR");
      } else if (errno == EINVAL) {
        throw std::runtime_error(error_msg + "EINVAL");
      } else if (errno == EIO) {
        throw std::runtime_error(error_msg + "EIO");
      } else if (errno == EISDIR) {
        throw std::runtime_error(error_msg + "EISDIR");
      } else {
        throw std::runtime_error("Error in the B3mPort::read_. read errorno: " +
                                 std::to_string(errno));
      }
    }
  }
}

tcflag_t B3mPort::getCBAUD() {
  switch (baudrate_) {
    case 4000000:
      return B4000000;
      break;
    case 3500000:
      return B3500000;
      break;
    case 3000000:
      return B3000000;
      break;
    case 2500000:
      return B2500000;
      break;
    case 2000000:
      return B2000000;
      break;
    case 1500000:
      return B1500000;
      break;
    case 1152000:
      return B1152000;
      break;
    case 1000000:
      return B1000000;
      break;
    case 921600:
      return B921600;
      break;
    case 576000:
      return B576000;
      break;
    case 500000:
      return B500000;
      break;
    case 460800:
      return B460800;
      break;
    case 230400:
      return B230400;
      break;
    case 115200:
      return B115200;
      break;
    case 57600:
      return B57600;
      break;
    case 38400:
      return B38400;
      break;
    case 19200:
      return B19200;
      break;
    case 9600:
      return B9600;
      break;
    case 4800:
      return B4800;
      break;
    case 2400:
      return B2400;
      break;
    case 1800:
      return B1800;
      break;
    case 1200:
      return B1200;
      break;
    case 600:
      return B600;
      break;
    case 300:
      return B300;
      break;
    case 200:
      return B200;
      break;
    case 150:
      return B150;
      break;
    case 134:
      return B134;
      break;
    case 110:
      return B110;
      break;
    case 75:
      return B75;
      break;
    case 50:
      return B50;
      break;
    default:
      return B0;
      break;
  }
}

timespec B3mPort::getGuardTime() {
  timespec t;
  t.tv_sec  = 0;
  t.tv_nsec = std::ceil(16.0 * 1000000000 / baudrate_) + (220 * 1000);
  return t;
}
