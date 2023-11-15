#ifndef KONDO_B3M_UTIL_HPP_
#define KONDO_B3M_UTIL_HPP_

#include <chrono>
#include <vector>
#include "b3m_motor.hpp"
#include "kondo_b3m_interfaces/srv/control_mode.hpp"
#include "kondo_b3m_interfaces/srv/get_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

class KondoB3mUtil : public rclcpp::Node
{
public:
  KondoB3mUtil();

private:
  int publish_frequency_;
  std::vector<B3mMotor> motor_list_;

  rclcpp::CallbackGroup::SharedPtr group_;

  rclcpp::Client<kondo_b3m_interfaces::srv::ControlMode>::SharedPtr client_mode_;
  rclcpp::Client<kondo_b3m_interfaces::srv::GetState>::SharedPtr client_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

  void cb_timer_();
};

#endif  // KONDO_B3M_UTIL_HPP_
