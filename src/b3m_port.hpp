#ifndef B3M_PORT_HPP_
#define B3M_PORT_HPP_

#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define B3M_COMMAND_MAX_LENGTH 256

class B3mPort
{
public:
  B3mPort(std::string device_name, uint32_t baudrate);
  ~B3mPort();
  bool commandLoad(uint8_t *id, uint8_t num);
  bool commandSave(uint8_t *id, uint8_t num);
  bool commandRead(uint8_t id, uint8_t address, uint8_t length, uint8_t *buf);
  bool commandWrite(uint8_t *id, uint8_t num, uint8_t *data, uint8_t data_length, uint8_t address);
  bool commandReset(uint8_t *id, uint8_t num);
  bool commandPosition(uint8_t *id, uint8_t num, uint8_t *pos, uint8_t *time);

private:
  uint32_t baudrate_;
  bool initialized_;
  std::string device_name_;
  int device_file_;

  bool sendCommand(uint8_t *command, uint8_t command_length);
  bool sendCommand(uint8_t *command, uint8_t command_length, uint8_t *buf, uint8_t buf_length);
  int readPort(uint8_t *buf, uint8_t count);
  bool writePort(uint8_t *buf, uint8_t count);
  void clearBuffer(void);
  uint8_t calc_checksum(uint8_t *command, uint8_t com_len);
  tcflag_t getCBAUD();
};

#endif // B3M_PORT_HPP_