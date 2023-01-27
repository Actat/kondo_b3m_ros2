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
  fd_set set;
  FD_ZERO(&set);
  FD_SET(device_file_, &set);
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 5 * 1000;
  int s           = select(device_file_ + 1, &set, NULL, NULL, &timeout);
  if (s == 0) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Failed to read due to timeout.");
    return B3mCommand();
  } else if (s == -1) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Error in the read_device function. (select errorno: %d)",
                errno);
    return B3mCommand();
  }

  std::vector<unsigned char> buf;
  if (!read_(buf.data(), 1)) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Failed to read command. (size)");
    return B3mCommand();
  }
  buf.resize(buf.at(0));
  timespec rem = guard_time_;
  bool result  = read_(buf.data() + 1, buf.at(0) - 1);
  while (nanosleep(&rem, &rem) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Error in the read_device function. (nanosleep errorno: %d)",
                errno);
  }
  if (!result) {
    RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                "Failed to read command. (data)");
    return B3mCommand();
  }
  return B3mCommand(buf);
}

bool B3mPort::read_(unsigned char *buf, size_t nbytes) {
  size_t rbytes = 0;
  while (rbytes < nbytes) {
    ssize_t r = read(device_file_, buf + rbytes, nbytes);
    if (r > 0) {
      rbytes += r;
    } else if (r == 0) {
      RCLCPP_WARN(rclcpp::get_logger("B3mPort"), "Failed to read due to EOF.");
      return false;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("B3mPort"),
                  "Error in the B3mPort::read_. (read errorno: %d)", errno);
      return false;
    }
  }
  return true;
}

/*
bool B3mPort::commandLoad(uint8_t id_len, uint8_t *id) {
  int command_len = id_len + 4;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  std::vector<uint8_t> command(command_len, 0);
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x01;                  // COMMAND
  command[2] = getOptionByte(id[0]);  // OPTION
  // ID
  for (uint8_t i = 0; i < id_len; i++) {
    command[i + 3] = id[i];
  }
  command[command_len - 1] = calc_checksum(command);
  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command, false);
  } else {
    // single mode
    return sendCommand(command, true) && (readCommand(command).size() == 5);
  }
}

bool B3mPort::commandSave(uint8_t id_len, uint8_t *id) {
  int command_len = id_len + 4;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  std::vector<uint8_t> command(command_len, 0);
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x02;                  // COMMAND
  command[2] = getOptionByte(id[0]);  // OPTION
  // ID
  for (uint8_t i = 0; i < id_len; i++) {
    command[i + 3] = id[i];
  }
  command[command_len - 1] = calc_checksum(command);
  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command, false);
  } else {
    // single mode
    return sendCommand(command, true) && (readCommand(command).size() == 5);
  }
}

bool B3mPort::commandRead(uint8_t id,
                          uint8_t address,
                          uint8_t length,
                          uint8_t *buf) {
  if (id == 0xFF || length < 0x01 || length > 0xFA) {
    return false;
  }

  std::vector<uint8_t> command(7);
  command[0] = 7;                  // SIZE
  command[1] = 0x03;               // COMMAND
  command[2] = getOptionByte(id);  // OPTION
  command[3] = id;
  command[4] = address;
  command[5] = length;
  command[6] = calc_checksum(command);

  if (!sendCommand(command, true)) {
    return false;
  }
  std::vector<uint8_t> tmp_buf = readCommand(command);
  if (tmp_buf.size() < (unsigned)(length + 5)) {
    return false;
  }
  for (uint8_t i = 0; i < length; i++) {
    buf[i] = tmp_buf[i + 4];
  }
  return true;
}

bool B3mPort::commandWrite(uint8_t id_len,
                           uint8_t *id,
                           uint8_t data_len,
                           uint8_t *data,
                           uint8_t address) {
  int command_len = id_len * (data_len + 1) + 6;
  if (id_len <= 0 || data_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  std::vector<uint8_t> command(command_len, 0);
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x04;                  // COMMAND
  command[2] = getOptionByte(id[0]);  // OPTION
  // ID and data
  for (uint8_t i = 0; i < id_len; i++) {
    command[i * (data_len + 1) + 3] = id[i];
    for (uint8_t j = 0; j < data_len; j++) {
      command[i * (data_len + 1) + 4 + j] = data[i * (data_len) + j];
    }
  }
  command[command_len - 3] = address;
  command[command_len - 2] = id_len;
  command[command_len - 1] = calc_checksum(command);
  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command, false);
  } else {
    // single mode
    return sendCommand(command, true) && (readCommand(command).size() == 5);
  }
}

bool B3mPort::commandReset(uint8_t id_len, uint8_t *id) {
  int command_len = id_len + 5;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  std::vector<uint8_t> command(command_len, 0);
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x05;                  // COMMAND
  command[2] = getOptionByte(id[0]);  // OPTION
  // ID
  for (uint8_t i = 0; i < id_len; i++) {
    command[i + 3] = id[i];
  }
  command[command_len - 2] = 0x03;  // TIME (reset immediately)
  command[command_len - 1] = calc_checksum(command);

  return sendCommand(command, false);
}

bool B3mPort::commandPosition(uint8_t id_len,
                              uint8_t *id,
                              uint8_t *pos,
                              uint8_t *time) {
  int command_len = id_len * 3 + 9;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  std::vector<uint8_t> command(command_len, 0);
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x06;                  // COMMAND
  command[2] = getOptionByte(id[0]);  // OPTION
  // ID and pos
  for (uint8_t i = 0; i < id_len; i++) {
    command[3 * i + 3] = id[i];
    command[3 * i + 4] = pos[i * 2];
    command[3 * i + 5] = pos[i * 2 + 1];
  }
  command[command_len - 3] = time[0];
  command[command_len - 2] = time[1];
  command[command_len - 1] = calc_checksum(command);

  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command, false);
  } else {
    // single mode
    return sendCommand(command, true) && (readCommand(command).size() == 7);
  }
}

std::vector<bool> B3mPort::commandMultiMotorRead(uint8_t id_len,
                                                 uint8_t *id,
                                                 uint8_t address,
                                                 uint8_t length,
                                                 uint8_t *buf) {
  std::vector<bool> is_sent(id_len);
  std::vector<bool> is_success(id_len);
  std::vector<std::vector<uint8_t>> coms(id_len);
  for (int i = 0; i < id_len; i++) {
    std::vector<uint8_t> command(7);
    command[0] = 7;                     // SIZE
    command[1] = 0x03;                  // COMMAND
    command[2] = getOptionByte(id[i]);  // OPTION
    command[3] = id[i];
    command[4] = address;
    command[5] = length;
    command[6] = calc_checksum(command);
    is_sent[i] = sendCommand(command, true);
    coms[i]    = command;
  }
  for (int i = 0; i < id_len; i++) {
    if (!is_sent[i]) {
      is_success[i] = false;
      continue;
    }
    std::vector<uint8_t> tmp_buf = readCommand(coms[i]);
    if (tmp_buf.size() != (unsigned)(length + 5)) {
      is_success[i] = false;
      continue;
    }
    is_success[i] = true;
    for (int j = 0; j < length; j++) {
      buf[i * length + j] = tmp_buf[j + 4];
    }
  }
  return is_success;
}
*/

// private---------------------------------------------------------------------

/*
bool B3mPort::sendCommand(std::vector<uint8_t> command, bool expect_reply) {
  if (expect_reply) {
    uint16_t key = ((command[1] | 0x80) << 8) | command[3];
    if (commands_.find(key) != commands_.end()) {
      return false;
    } else {
      std::vector<uint8_t> v;
      v.push_back(command[2]);
      commands_.insert(std::make_pair(key, v));
    }
  }

  if (is_busy_) {
    return false;
  }
  is_busy_        = true;
  bool is_written = writePort(command.size(), &command[0]);
  is_busy_        = false;

  return is_written;
}

std::vector<uint8_t> B3mPort::readCommand(std::vector<uint8_t> command) {
  using namespace std::chrono_literals;

  if (!initialized_) {
    std::vector<uint8_t> v = {};
    return v;
  }

  uint16_t key = ((command[1] | 0x80) << 8) | command[3];
  if (commands_[key].size() <= 1) {
    int timeout_ns = 100 * 1000;
    auto t1        = rclcpp::Clock().now();
    while (true) {
      readStream();

      if (commands_[key].size() != 0) {
        break;
      }

      auto t2 = rclcpp::Clock().now();
      if ((t2 - t1).nanoseconds() > timeout_ns) {
        RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"), "Timeout (readCommand)");
        commands_.erase(key);
        std::vector<uint8_t> v = {};
        return v;
      }
    }
  }

  std::vector<uint8_t> v = commands_[key];
  commands_.erase(key);

  return v;
}

void B3mPort::readStream() {
  using namespace std::chrono_literals;

  fd_set set;
  FD_ZERO(&set);
  FD_SET(device_file_, &set);
  struct timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 100 * 1000;
  int s           = select(device_file_ + 1, &set, NULL, NULL, &timeout);
  if (s < 0) {
    throw std::runtime_error("Read error. Can not access file. errno: " +
                             std::to_string(errno));
  } else if (s == 0) {
    // timeout
    return;
  } else {
    while (true) {
      uint8_t b;
      int s = read(device_file_, &b, 1);
      if (s != 1) {
        return;
      }
      std::vector<uint8_t> buf(b);
      buf[0] = b;

      auto t1 = rclcpp::Clock().now();
      while (true) {
        int size;
        ioctl(device_file_, FIONREAD, &size);
        if (size >= b - 1) {
          break;
        }

        auto t2 = rclcpp::Clock().now();
        if ((t2 - t1).nanoseconds() > 800000000) {
          clearBuffer();
          commands_.clear();
          RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                      "Failed to receive command. Buffer cleared.");
          return;
        }
      }

      ssize_t n_bytes_read = read(device_file_, &buf[1], b - 1);
      if (n_bytes_read < 0) {
        throw std::runtime_error("Read error. errno: " + std::to_string(errno));
      }

      inspectCommand(buf);

      uint16_t key   = (buf[1] << 8) | buf[3];
      commands_[key] = buf;

      int size;
      ioctl(device_file_, FIONREAD, &size);
      if (size < 1) {
        break;
      }
    }
  }
}

void B3mPort::inspectCommand(std::vector<uint8_t> command) {
  if (command.back() != calc_checksum(command)) {
    RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                "Invalid command is found. (Checksum)");
    return;
  }

  uint8_t id               = command[3];
  uint16_t key             = (command[1] << 8) | command[3];
  uint8_t sent_option_byte = commands_[key][0] & 0b00000111;
  uint8_t received_status =
      (commands_[key][0] & (1 << 7)) != 0 ? 0 : command[2];

  if (sent_option_byte == 0b000) {
    if (status_bytes_.find(id) == status_bytes_.end()) {
      std::vector<uint8_t> v = {received_status, 0};
      status_bytes_.insert(std::make_pair(id, v));
    } else {
      status_bytes_[id][0] = received_status;
    }

    if ((received_status & 0b0001) == 0b0001) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) + ")");
    }

    if ((received_status & 0b0010) == 0b0010) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "MOTOR STATUS ERROR (ID: " + std::to_string(id) + ")");
    }

    if ((received_status & 0b0100) == 0b0100) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "UART STATUS ERROR (ID: " + std::to_string(id) + ")");
    }

    if ((received_status & 0b1000) == 0b1000) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "COMMAND STATUS ERROR (ID: " + std::to_string(id) + ")");
    }

  } else if (sent_option_byte == 0b001) {
    status_bytes_[id][1] |= (1 << 0);

    if ((received_status & (1 << 0)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): Watchdog Timer is started");
    }

    if ((received_status & (1 << 1)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): A problem with data saved in MCU ROM");
    }

    if ((received_status & (1 << 2)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): Problem with data and RAM allocation fails");
    }

    if ((received_status & (1 << 3)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): Input voltage exceeds maximum or is lower " +
                      "than minimum value");
    }

    if ((received_status & (1 << 4)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): MCU temperature exceeds maximum value");
    }

    if ((received_status & (1 << 5)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): AD conversion fails");
    }

    if ((received_status & (1 << 6)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): I2C communication fails");
    }

    if ((received_status & (1 << 7)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "SYSTEM STATUS ERROR (ID: " + std::to_string(id) +
                      "): SPI communication fails");
    }

  } else if (sent_option_byte == 0b010) {
    status_bytes_[id][1] |= (1 << 1);

    if ((received_status & (1 << 0)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "MOTOR STATUS ERROR (ID: " + std::to_string(id) +
                      "): Motor temperature exceeds maximum value");
    }

    if ((received_status & (1 << 1)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "MOTOR STATUS ERROR (ID: " + std::to_string(id) +
                      "): Motor lock detected");
    }

    if ((received_status & (1 << 2)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "MOTOR STATUS ERROR (ID: " + std::to_string(id) +
                      "): Current flowing to motor exceeds maximum value");
    }

    if ((received_status & (1 << 3)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "MOTOR STATUS ERROR (ID: " + std::to_string(id) +
                      "): Problem with brushless motor's Hall-IC");
    }

  } else if (sent_option_byte == 0b011) {
    status_bytes_[id][1] |= (1 << 2);

    if ((received_status & (1 << 0)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "UART STATUS ERROR (ID: " + std::to_string(id) +
                      "): Framing error occurs");
    }

    if ((received_status & (1 << 1)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "UART STATUS ERROR (ID: " + std::to_string(id) +
                      "): Parity error occurs");
    }

    if ((received_status & (1 << 2)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "UART STATUS ERROR (ID: " + std::to_string(id) +
                      "): Break error occurs");
    }

    if ((received_status & (1 << 3)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "UART STATUS ERROR (ID: " + std::to_string(id) +
                      "): Overrun error occurs");
    }

  } else if (sent_option_byte == 0b100) {
    status_bytes_[id][1] |= (1 << 3);

    if ((received_status & (1 << 0)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "COMMAND STATUS ERROR (ID: " + std::to_string(id) +
                      "): Problem with command checksum");
    }

    if ((received_status & (1 << 1)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "COMMAND STATUS ERROR (ID: " + std::to_string(id) +
                      "): Command device number is too many or too few");
    }

    if ((received_status & (1 << 2)) != 0) {
      RCLCPP_WARN(
          rclcpp::get_logger("kondo_b3m"),
          "COMMAND STATUS ERROR (ID: " + std::to_string(id) +
              "): Length of data to be acquired is longer than address");
    }

    if ((received_status & (1 << 3)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "COMMAND STATUS ERROR (ID: " + std::to_string(id) +
                      "): Address out of specified range");
    }

    if ((received_status & (1 << 4)) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                  "COMMAND STATUS ERROR (ID: " + std::to_string(id) +
                      "): Problem with command itself");
    }

  } else {
    RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
                "COMMAND ERROR (Invalid status byte)");
  }
}

bool B3mPort::writePort(uint8_t buf_len, uint8_t *buf) {
  if (!initialized_) {
    return false;
  }

  ssize_t written = write(device_file_, buf, buf_len);
  rclcpp::sleep_for(guard_time_);

  if (written >= 0) {
    // success
    return true;
  } else if (errno == EWOULDBLOCK) {
    // write is blocked
    return false;
  } else {
    return false;
  }
}
*/

/*
void B3mPort::clearBuffer(void) {
  int size;
  uint8_t buf;

  ioctl(device_file_, FIONREAD, &size);
  for (int i = 0; i < size; i++) {
    read(device_file_, &buf, 1);
  }
  return;
}
*/

/*
uint8_t B3mPort::getOptionByte(uint8_t id) {
  if (status_bytes_.find(id) == status_bytes_.end()) {
    return 0;
  }

  uint8_t preserved_status = status_bytes_[id][0];
  uint8_t checked_status   = status_bytes_[id][1];

  if (preserved_status == 0) {
    return 0;
  }

  if (checked_status == preserved_status) {
    RCLCPP_INFO(rclcpp::get_logger("kondo_b3m"),  //
                "Clear status error. (id: %d)", id);
    status_bytes_.erase(id);
    return (1 << 7);
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (((preserved_status ^ checked_status) & (1 << i)) != 0) {
      return i + 1;
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("kondo_b3m"),
              "Preserved status is broken. (%d, %d)",  //
              preserved_status, checked_status);
  return (1 << 0);
}

uint8_t B3mPort::calc_checksum(std::vector<uint8_t> command) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < command.size() - 1; i++) {
    sum += command[i];
  }
  return sum;
}
*/

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
