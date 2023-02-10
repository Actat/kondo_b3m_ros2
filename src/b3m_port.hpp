#ifndef B3M_PORT_HPP_
#define B3M_PORT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "b3m_command.hpp"
// --- pigpio ---
#include <pigpiod_if2.h>
// --- pigpio ---

int const B3M_COMMAND_MAX_LENGTH = 256;

class B3mPort {
public:
  B3mPort(std::string device_name, uint32_t baudrate);
  virtual ~B3mPort();
  bool wright_device(B3mCommand const &command);
  B3mCommand read_device();

protected:
  uint32_t baudrate_;
  bool initialized_;
  std::string device_name_;
  int device_file_;
  timespec guard_time_;

  void read_(unsigned char *buf, size_t nbytes);
  tcflag_t getCBAUD();
  timespec getGuardTime();
  virtual void setEN(bool bit) {
    (void)bit;
    return;
  };
};

// --- pigpio ---
class B3mPigpio : public B3mPort {
public:
  B3mPigpio(std::string device_name, uint32_t baudrate)
      : B3mPort(device_name, baudrate) {
    pigpio_ = pigpio_start(NULL, NULL);
    if (pigpio_ < 0) {
      initialized_ = false;
      throw std::runtime_error("gpioInitialize() failed");
    }
    RCLCPP_INFO(rclcpp::get_logger("B3mPort"), "pigpio started.");
  };

  ~B3mPigpio() {
    pigpio_stop(pigpio_);
    RCLCPP_INFO(rclcpp::get_logger("B3mPort"), "pigpio stopped.");
  }

private:
  int const EN_PIN = 25;
  int pigpio_;
  virtual void setEN(bool bit) override {
    switch (bit ? gpio_write(pigpio_, EN_PIN, 1)
                : gpio_write(pigpio_, EN_PIN, 0)) {
      case PI_BAD_GPIO:
        RCLCPP_WARN(rclcpp::get_logger("B3mPigpio"),
                    "senEn failed: PI_BAD_GPIO");
        break;
      case PI_BAD_LEVEL:
        RCLCPP_WARN(rclcpp::get_logger("B3mPigpio"),
                    "senEn failed: PI_BAD_LEVEL");
        break;
      case PI_NOT_PERMITTED:
        RCLCPP_WARN(rclcpp::get_logger("B3mPigpio"),
                    "senEn failed: PI_NOT_PERMITTED");
        break;

      default:
        break;
    }
  }
};
// --- pigpio ---

#endif  // B3M_PORT_HPP_
