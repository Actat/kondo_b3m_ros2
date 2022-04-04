#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define B3M_COMMAND_MAX_LENGTH 256
#define B3M_DATA_MAX_LENGTH 251 // B3M_COMMAND_MAX_LENGTH - length of (SIZE, COMMAND, OPTION, ID, SUM)

class B3mPort
{
public:
  B3mPort(std::string device_name, uint32_t baudrate);
  ~B3mPort();
  bool commandLoad(uint8_t *id, uint8_t num);
  bool commandSave(uint8_t *id, uint8_t num);
  bool commandRead(uint8_t id, uint8_t address, uint8_t length);
  bool commandWrite(uint8_t *id, uint8_t num, uint8_t *data, uint8_t data_length, uint8_t address);
  bool commandReset(uint8_t *id, uint8_t num);
  bool commandPosition(uint8_t *id, uint8_t num);

private:
  uint32_t baudrate_;
  bool initialized_;
  std::string device_name_;
  int device_file_;

  int readPort(uint8_t *buf, uint8_t count);
  bool writePort(uint8_t *buf, uint8_t count);
  void clearBuffer(void);
  uint8_t calc_checksum(uint8_t *command, uint8_t com_len);
  tcflag_t getCBAUD();
};

B3mPort::B3mPort(std::string device_name, uint32_t baudrate)
{
  initialized_ = false;
  baudrate_ = baudrate;
  device_name_ = device_name;
  device_file_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (device_file_ < 0)
  {
    throw std::runtime_error("Could not open device file: " + device_name_ + ": " + std::to_string(device_file_));
  }
  tcflag_t baud = getCBAUD();
  if (baud == B0)
  {
    throw std::runtime_error("invalid baudrate");
  }
  struct termios tio;
  tio.c_iflag = IGNPAR;
  tio.c_oflag = 0;
  tio.c_cflag = baud | CSTOPB | CREAD | CLOCAL;
  tio.c_lflag = 0;
  tio.c_line = 0;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tcflush(device_file_, TCIFLUSH);
  tcsetattr(device_file_, TCSANOW, &tio);
  clearBuffer();
  initialized_ = true;
  return;
}

B3mPort::~B3mPort()
{
  if (initialized_)
  {
    close(device_file_);
    initialized_ = false;
  }
}

bool B3mPort::commandLoad(uint8_t *id, uint8_t num)
{
  if (num <= 0 || num > B3M_DATA_MAX_LENGTH)
  {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = num + 4;    // SIZE
  command[1] = 0x01;       // COMMAND
  command[2] = 0b10000000; // OPTION (STATUS CLEAR)
  // ID
  for (uint8_t i = 0; i < num; i++)
  {
    command[i + 3] = id[i];
  }
  command[num + 3] = calc_checksum(command, num + 4);
  return writePort(command, num + 4);
}

bool B3mPort::commandSave(uint8_t *id, uint8_t num)
{
  if (num <= 0 || num > B3M_DATA_MAX_LENGTH)
  {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = num + 4;    // SIZE
  command[1] = 0x02;       // COMMAND
  command[2] = 0b10000000; // OPTION (STATUS CLEAR)
  // ID
  for (uint8_t i = 0; i < num; i++)
  {
    command[i + 3] = id[i];
  }
  command[num + 3] = calc_checksum(command, num + 4);
  return writePort(command, num + 4);
}

bool B3mPort::commandRead(uint8_t id, uint8_t address, uint8_t length)
{
  if (id > 0xFE || length < 0x01 || length > 0xFA)
  {
    return false;
  }

  uint8_t command[7];
  command[0] = 7;          // SIZE
  command[1] = 0x03;       // COMMAND
  command[2] = 0b10000000; // OPTION (STATUS CLEAR)
  command[3] = id;
  command[4] = address;
  command[5] = length;
  command[6] = calc_checksum(command, 7);
  return writePort(command, 7);
}

bool B3mPort::commandWrite(uint8_t *id, uint8_t num, uint8_t *data, uint8_t data_length, uint8_t address)
{
  if (num * (data_length + 1) + 6 > B3M_COMMAND_MAX_LENGTH)
  {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = num * (data_length + 1) + 6; // SIZE
  command[1] = 0x04;                        // COMMAND
  command[2] = 0b00000000;                  // OPTION
  // ID and data
  for (uint8_t i = 0; i < num; i++)
  {
    command[i * (data_length + 1) + 3] = id[i];
    for (uint8_t j = 0; j < data_length; j++)
    {
      command[i * (data_length + 1) + 4 + j] = data[i * (data_length) + j];
    }
  }
  command[num * (data_length + 1) + 3] = address;
  command[num * (data_length + 1) + 4] = num;
  command[num * (data_length + 1) + 5] = calc_checksum(command, num * (data_length + 1) + 6);
  if (writePort(command, num * (data_length + 1) + 6))
  {
    if (num == 1)
    {
      uint8_t buf[5];
      int read = readPort(buf, 5);
      if (read == 5)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool B3mPort::commandReset(uint8_t *id, uint8_t num)
{
  if (num <= 0 || num > B3M_DATA_MAX_LENGTH)
  {
    return false;
  }

  uint8_t command[B3M_COMMAND_MAX_LENGTH];
  command[0] = num + 5;    // SIZE
  command[1] = 0x05;       // COMMAND
  command[2] = 0b10000000; // OPTION (STATUS CLEAR)
  // ID
  for (uint8_t i = 0; i < num; i++)
  {
    command[i + 3] = id[i];
  }
  command[num + 3] = 0x03; // TIME (reset immediately)
  command[num + 4] = calc_checksum(command, num + 5);
  return writePort(command, num + 5);
}

int B3mPort::readPort(uint8_t *buf, uint8_t count)
{
  if (!initialized_)
  {
    return -1;
  }
  fd_set set;
  FD_ZERO(&set);
  FD_SET(device_file_, &set);
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 100 * 1000;
  int s = select(device_file_ + 1, &set, NULL, NULL, &timeout);
  if (s < 0)
  {
    throw std::runtime_error("Read error. Can not access file. errno: " + std::to_string(errno));
  }
  else if (s == 0)
  {
    // timeout
    return -2;
  }
  else
  {
    int size;
    ioctl(device_file_, FIONREAD, &size);
    if (size < count)
    {
      usleep(2000);
      ioctl(device_file_, FIONREAD, &size);
    }
    ssize_t n_bytes_read = read(device_file_, buf, size);
    if (n_bytes_read < 0)
    {
      throw std::runtime_error("Read error. errno: " + std::to_string(errno));
    }
    else
    {
      // number of bytes read is less than 'count'
      return (int)n_bytes_read;
    }
  }
}

bool B3mPort::writePort(uint8_t *buf, uint8_t count)
{
  if (!initialized_)
  {
    return false;
  }
  ssize_t written = write(device_file_, buf, count);
  if (written >= 0)
  {
    // success
    return true;
  }
  else if (errno == EWOULDBLOCK)
  {
    // write is blocked
    return false;
  }
  else
  {
    return false;
  }
}

void B3mPort::clearBuffer(void)
{
  int size;
  uint8_t buf;

  ioctl(device_file_, FIONREAD, &size);
  for (int i = 0; i < size; i++)
  {
    read(device_file_, &buf, 1);
  }
  return;
}

tcflag_t B3mPort::getCBAUD()
{
  switch (baudrate_)
  {
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

uint8_t B3mPort::calc_checksum(uint8_t *command, uint8_t com_len)
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < com_len - 1; i++)
  {
    sum += command[i];
  }
  return sum;
}
