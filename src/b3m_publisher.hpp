#ifndef B3M_PUBLISHER_HPP_
#define B3M_PUBLISHER_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class B3mPublisher : public rclcpp::Node
{
public:
  B3mPublisher();

private:
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

#endif // B3M_PUBLISHER_HPP_
