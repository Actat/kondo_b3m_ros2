#ifndef KONDO_B3M_HPP_
#define KONDO_B3M_HPP_

#include <algorithm>
#include <chrono>
#include <vector>
#include "b3m_motor.hpp"
#include "b3m_port.hpp"
#include "kondo_b3m_interfaces/msg/set_desired.hpp"
#include "kondo_b3m_interfaces/srv/control_mode.hpp"
#include "kondo_b3m_interfaces/srv/desired.hpp"
#include "kondo_b3m_interfaces/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

class KondoB3m : public rclcpp::Node
{
public:
  KondoB3m();
  ~KondoB3m();

private:
  std::string port_name_;
  uint32_t baudrate_;
  int publish_frequency_;
  B3mPort * port_;
  std::vector<B3mMotor> motor_list_;
  rclcpp::Subscription<kondo_b3m_interfaces::msg::SetDesired>::SharedPtr sub_pos_;
  rclcpp::Subscription<kondo_b3m_interfaces::msg::SetDesired>::SharedPtr sub_vel_;
  rclcpp::Service<kondo_b3m_interfaces::srv::ControlMode>::SharedPtr
    service_control_mode_;
  rclcpp::Service<kondo_b3m_interfaces::srv::Desired>::SharedPtr service_desired_;
  rclcpp::Service<kondo_b3m_interfaces::srv::GetState>::SharedPtr service_state_;

  void set_pos_(kondo_b3m_interfaces::msg::SetDesired::SharedPtr const msg);
  void set_vel_(kondo_b3m_interfaces::msg::SetDesired::SharedPtr const msg);
  void control_mode_(
    std::shared_ptr<kondo_b3m_interfaces::srv::ControlMode::Request> const request,
    std::shared_ptr<kondo_b3m_interfaces::srv::ControlMode::Response> response);
  void desired_(
    std::shared_ptr<kondo_b3m_interfaces::srv::Desired::Request> const request,
    std::shared_ptr<kondo_b3m_interfaces::srv::Desired::Response> response);
  void state_(
    std::shared_ptr<kondo_b3m_interfaces::srv::GetState::Request> const request,
    std::shared_ptr<kondo_b3m_interfaces::srv::GetState::Response> response);
  B3mCommand send_command_(B3mCommand const & command);
};

#endif  // KONDO_B3M_HPP_
