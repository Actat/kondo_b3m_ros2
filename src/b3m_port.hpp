#ifndef B3M_PORT_HPP_
#define B3M_PORT_HPP_

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#define B3M_COMMAND_MAX_LENGTH 256

class B3mPort {
public:
  B3mPort(std::string device_name, uint32_t baudrate);
  ~B3mPort();
  bool commandLoad(uint8_t id_len, uint8_t *id);
  bool commandSave(uint8_t id_len, uint8_t *id);
  bool commandRead(uint8_t id, uint8_t address, uint8_t length, uint8_t *buf);
  bool commandWrite(uint8_t id_len,
                    uint8_t *id,
                    uint8_t data_len,
                    uint8_t *data,
                    uint8_t address);
  bool commandReset(uint8_t id_len, uint8_t *id);
  bool commandPosition(uint8_t id_len,
                       uint8_t *id,
                       uint8_t *pos,
                       uint8_t *time);
  std::vector<bool> commandMultiMotorRead(uint8_t id_len,
                                          uint8_t *id,
                                          uint8_t address,
                                          uint8_t length,
                                          uint8_t *buf);

private:
  uint32_t baudrate_;
  bool initialized_;
  std::string device_name_;
  int device_file_;
  bool is_busy_;
  std::chrono::microseconds guard_time_;

  bool sendCommand(uint8_t com_len, uint8_t *command);
  bool sendCommand(uint8_t com_len,
                   uint8_t *command,
                   uint8_t buf_len,
                   uint8_t *buf);
  int readPort(uint8_t buf_len, uint8_t *buf);
  void readStream();
  bool writePort(uint8_t buf_len, uint8_t *buf);
  void clearBuffer(void);
  uint8_t calc_checksum(uint8_t com_len, uint8_t *command);
  tcflag_t getCBAUD();
  std::chrono::microseconds getGuardTime();
};

#endif  // B3M_PORT_HPP_
