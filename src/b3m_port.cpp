#include "b3m_port.hpp"

B3mPort::B3mPort(std::string device_name, uint32_t baudrate) {
  is_busy_     = true;
  initialized_ = false;
  baudrate_    = baudrate;
  device_name_ = device_name;
  guard_time_  = getGuardTime();
  device_file_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (device_file_ < 0) {
    throw std::runtime_error("Could not open device file: " + device_name_ +
                             ": " + std::to_string(device_file_));
  }
  tcflag_t baud = getCBAUD();
  if (baud == B0) {
    throw std::runtime_error("invalid baudrate");
  }
  struct termios tio;
  tio.c_iflag     = IGNPAR;
  tio.c_oflag     = 0;
  tio.c_cflag     = baud | CSTOPB | CREAD | CLOCAL;
  tio.c_lflag     = 0;
  tio.c_line      = 0;
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;
  tcflush(device_file_, TCIFLUSH);
  tcsetattr(device_file_, TCSANOW, &tio);
  clearBuffer();
  initialized_ = true;
  is_busy_     = false;
  return;
}

B3mPort::~B3mPort() {
  if (initialized_) {
    close(device_file_);
    initialized_ = false;
  }
}

bool B3mPort::commandLoad(uint8_t id_len, uint8_t *id) {
  int command_len = id_len + 4;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x01;                  // COMMAND
  command[2] = 0x00;                  // OPTION
  // ID
  for (uint8_t i = 0; i < id_len; i++) {
    command[i + 3] = id[i];
  }
  command[command_len - 1] = calc_checksum(command_len, command);
  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command_len, command);
  } else {
    // single mode
    uint8_t buf[5];
    return sendCommand(command_len, command, 5, buf);
  }
}

bool B3mPort::commandSave(uint8_t id_len, uint8_t *id) {
  int command_len = id_len + 4;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x02;                  // COMMAND
  command[2] = 0x00;                  // OPTION
  // ID
  for (uint8_t i = 0; i < id_len; i++) {
    command[i + 3] = id[i];
  }
  command[command_len - 1] = calc_checksum(command_len, command);
  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command_len, command);
  } else {
    // single mode
    uint8_t buf[5];
    return sendCommand(command_len, command, 5, buf);
  }
}

bool B3mPort::commandRead(uint8_t id,
                          uint8_t address,
                          uint8_t length,
                          uint8_t *buf) {
  if (id == 0xFF || length < 0x01 || length > 0xFA) {
    return false;
  }

  uint8_t command[7];
  command[0] = 7;     // SIZE
  command[1] = 0x03;  // COMMAND
  command[2] = 0x00;  // OPTION
  command[3] = id;
  command[4] = address;
  command[5] = length;
  command[6] = calc_checksum(7, command);

  uint8_t tmp_buf[B3M_COMMAND_MAX_LENGTH];
  if (!sendCommand(7, command, length + 5, tmp_buf)) {
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

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x04;                  // COMMAND
  command[2] = 0x00;                  // OPTION
  // ID and data
  for (uint8_t i = 0; i < id_len; i++) {
    command[i * (data_len + 1) + 3] = id[i];
    for (uint8_t j = 0; j < data_len; j++) {
      command[i * (data_len + 1) + 4 + j] = data[i * (data_len) + j];
    }
  }
  command[command_len - 3] = address;
  command[command_len - 2] = id_len;
  command[command_len - 1] = calc_checksum(command_len, command);
  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command_len, command);
  } else {
    // single mode
    uint8_t buf[5];
    return sendCommand(command_len, command, 5, buf);
  }
}

bool B3mPort::commandReset(uint8_t id_len, uint8_t *id) {
  int command_len = id_len + 5;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x05;                  // COMMAND
  command[2] = 0x00;                  // OPTION
  // ID
  for (uint8_t i = 0; i < id_len; i++) {
    command[i + 3] = id[i];
  }
  command[command_len - 2] = 0x03;  // TIME (reset immediately)
  command[command_len - 1] = calc_checksum(command_len, command);

  return sendCommand(command_len, command);
}

bool B3mPort::commandPosition(uint8_t id_len,
                              uint8_t *id,
                              uint8_t *pos,
                              uint8_t *time) {
  int command_len = id_len * 3 + 9;
  if (id_len <= 0 || command_len > B3M_COMMAND_MAX_LENGTH) {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = (uint8_t)command_len;  // SIZE
  command[1] = 0x06;                  // COMMAND
  command[2] = 0x00;                  // OPTION
  // ID and pos
  for (uint8_t i = 0; i < id_len; i++) {
    command[3 * i + 3] = id[i];
    command[3 * i + 4] = pos[i * 2];
    command[3 * i + 5] = pos[i * 2 + 1];
  }
  command[command_len - 3] = time[0];
  command[command_len - 2] = time[1];
  command[command_len - 1] = calc_checksum(command_len, command);

  if (id_len > 1 || id[0] == 0xFF) {
    // no return: multi mode of brodecast
    return sendCommand(command_len, command);
  } else {
    // single mode
    uint8_t buf[7];
    return sendCommand(command_len, command, 7, buf);
  }
}

std::vector<bool> B3mPort::commandMultiMotorRead(uint8_t id_len,
                                                 uint8_t *id,
                                                 uint8_t address,
                                                 uint8_t length,
                                                 uint8_t *buf) {
  // TODO: is_busy_
  std::vector<bool> is_sent(id_len);
  std::vector<bool> is_success(id_len);
  for (int i = 0; i < id_len; i++) {
    uint8_t command[7];
    command[0] = 7;     // SIZE
    command[1] = 0x03;  // COMMAND
    command[2] = 0x00;  // OPTION
    command[3] = id[i];
    command[4] = address;
    command[5] = length;
    command[6] = calc_checksum(7, command);
    is_sent[i] = sendCommand(7, command);
  }
  for (int i = 0; i < id_len; i++) {
    if (!is_sent[i]) {
      is_success[i] = false;
      continue;
    }
    std::vector<uint8_t> tmp_buf(length + 5);
    is_success[i] = readPort(length + 5, &tmp_buf[0]);
    for (int j = 0; j < length; j++) {
      buf[i * length + j] = tmp_buf[j + 4];
    }
  }

  return is_success;
}

// private---------------------------------------------------------------------

bool B3mPort::sendCommand(uint8_t com_len, uint8_t *command) {
  if (is_busy_) {
    return false;
  }
  is_busy_    = true;
  bool result = writePort(com_len, command);
  rclcpp::sleep_for(guard_time_);
  is_busy_ = false;
  return result;
}

bool B3mPort::sendCommand(uint8_t com_len,
                          uint8_t *command,
                          uint8_t buf_len,
                          uint8_t *buf) {
  if (is_busy_) {
    return false;
  }
  is_busy_ = true;
  if (!writePort(com_len, command)) {
    is_busy_ = false;
    return false;
  }
  int len  = readPort(buf_len, buf);
  is_busy_ = false;
  return len == buf_len;
}

int B3mPort::readPort(uint8_t buf_len, uint8_t *buf) {
  if (!initialized_) {
    return -1;
  }
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
    return -2;
  } else {
    int size;
    ioctl(device_file_, FIONREAD, &size);
    if (size < buf_len) {
      usleep(2000);
      ioctl(device_file_, FIONREAD, &size);
    }
    ssize_t n_bytes_read =
        read(device_file_, buf, std::min(size, (int)buf_len));
    if (n_bytes_read < 0) {
      throw std::runtime_error("Read error. errno: " + std::to_string(errno));
    } else {
      // id_lenber of bytes read is less than 'count'
      return (int)n_bytes_read;
    }
  }
}

bool B3mPort::writePort(uint8_t buf_len, uint8_t *buf) {
  if (!initialized_) {
    return false;
  }
  ssize_t written = write(device_file_, buf, buf_len);
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

void B3mPort::clearBuffer(void) {
  int size;
  uint8_t buf;

  ioctl(device_file_, FIONREAD, &size);
  for (int i = 0; i < size; i++) {
    read(device_file_, &buf, 1);
  }
  return;
}

uint8_t B3mPort::calc_checksum(uint8_t com_len, uint8_t *command) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < com_len - 1; i++) {
    sum += command[i];
  }
  return sum;
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

std::chrono::microseconds B3mPort::getGuardTime() {
  using namespace std::chrono_literals;
  int time_2byte = std::ceil(16.0 * 1000000 / baudrate_);
  std::chrono::microseconds t{time_2byte};
  return t + 220us;
}
