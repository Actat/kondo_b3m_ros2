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
  // bool commandLoad(uint8_t id_len, uint8_t *id);
  // bool commandSave(uint8_t id_len, uint8_t *id);
  // bool commandRead(uint8_t id, uint8_t address, uint8_t length, uint8_t
  // *buf); bool commandWrite(uint8_t id_len,
  //                   uint8_t *id,
  //                   uint8_t data_len,
  //                   uint8_t *data,
  //                   uint8_t address);
  // bool commandReset(uint8_t id_len, uint8_t *id);
  // bool commandPosition(uint8_t id_len,
  //                      uint8_t *id,
  //                      uint8_t *pos,
  //                      uint8_t *time);
  // std::vector<bool> commandMultiMotorRead(uint8_t id_len,
  //                                         uint8_t *id,
  //                                         uint8_t address,
  //                                         uint8_t length,
  //                                         uint8_t *buf);
  bool wright_device(B3mCommand const &command);
  B3mCommand read_device();

private:
  uint32_t baudrate_;
  bool initialized_;
  std::string device_name_;
  int device_file_;
  timespec guard_time_;
  // std::map<uint8_t, std::vector<uint8_t>> status_bytes_;
  // std::map<int16_t, std::vector<uint8_t>> commands_;

  bool read_(unsigned char *buf, size_t nbytes);
  // bool sendCommand(std::vector<uint8_t> command, bool expect_reply);
  // std::vector<uint8_t> readCommand(std::vector<uint8_t> command);
  // void readStream();
  // void inspectCommand(std::vector<uint8_t> command);
  // bool writePort(uint8_t buf_len, uint8_t *buf);
  // void clearBuffer(void);
  // uint8_t getOptionByte(uint8_t id);
  // uint8_t calc_checksum(std::vector<uint8_t> command);
  tcflag_t getCBAUD();
  timespec getGuardTime();
};

#endif  // B3M_PORT_HPP_
