#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>

#define B3M_COMMAND_MAX_LENGTH 256
#define B3M_DATA_MAX_LENGTH 251 // B3M_COMMAND_MAX_LENGTH - length of (SIZE, COMMAND, OPTION, ID, SUM)

class B3mPort
{
public:
  B3mPort(std::string device_name);
  ~B3mPort();
  bool commandLoad(uint8_t *id, uint8_t num);
  bool commandSave(uint8_t *id, uint8_t num);
  bool commandRead(uint8_t id, uint8_t address, uint8_t length);
  bool commandWrite(uint8_t *id, uint8_t num);
  bool commandReset(uint8_t *id, uint8_t num);
  bool commandPosition(uint8_t *id, uint8_t num);

private:
  bool initialized_;
  std::string device_name_;
  int device_file_;

  int readPort(uint8_t *buf, uint8_t count);
  bool writePort(uint8_t *buf, uint8_t count);
  uint8_t calc_checksum(uint8_t *command, uint8_t com_len);
};

B3mPort::B3mPort(std::string device_name)
{
  initialized_ = false;
  device_name_ = device_name;
  device_file_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (device_file_ < 0)
  {
    throw std::runtime_error("Could not open device file: " + device_name_ + ": " + std::to_string(device_file_));
  }
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
  command[2] = 0b00000000; // OPTION (STATUS CLEAR)
  // ID
  for (uint8_t i = 0; i < num; i++)
  {
    command[i + 3] = id[i];
  }
  command[num + 3] = this->calc_checksum(command, num + 4);
  return this->writePort(command, num + 4);
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
  command[2] = 0b00000000; // OPTION (STATUS CLEAR)
  // ID
  for (uint8_t i = 0; i < num; i++)
  {
    command[i + 3] = id[i];
  }
  command[num + 3] = this->calc_checksum(command, num + 4);
  return this->writePort(command, num + 4);
}

bool B3mPort::commandRead(uint8_t id, uint8_t address, uint8_t length)
{
  if (id < 0x00 || id > 0xFE || length < 0x01 || length > 0xFA)
  {
    return false;
  }

  uint8_t command[7];
  command[0] = 7;          // SIZE
  command[1] = 0x03;       // COMMAND
  command[2] = 0b00000000; // OPTION (STATUS CLEAR)
  command[3] = id;
  command[4] = address;
  command[5] = length;
  command[6] = this->calc_checksum(command, 7);
  return this->writePort(command, 7);
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
  command[2] = 0b00000000; // OPTION (STATUS CLEAR)
  // ID
  for (uint8_t i = 0; i < num; i++)
  {
    command[i + 3] = id[i];
  }
  command[num + 3] = 0x03; // TIME (reset immediately)
  command[num + 4] = this->calc_checksum(command, num + 5);
  return this->writePort(command, num + 5);
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
  timeout.tv_sec = 10;
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
    ssize_t n_bytes_read = read(device_file_, buf, count);
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

uint8_t B3mPort::calc_checksum(uint8_t *command, uint8_t com_len)
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < com_len - 1; i++)
  {
    sum += command[i];
  }
  return sum;
}
