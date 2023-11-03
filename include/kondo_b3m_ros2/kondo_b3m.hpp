#ifndef KONDO_B3M_HPP_
#define KONDO_B3M_HPP_

#include <algorithm>
#include <chrono>
#include <vector>
#include "b3m_motor.hpp"
#include "b3m_port.hpp"
#include "kondo_b3m_ros2/srv/control_mode.hpp"
#include "kondo_b3m_ros2/srv/desired.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Service<kondo_b3m_ros2::srv::ControlMode>::SharedPtr
    service_control_mode_;
  rclcpp::Service<kondo_b3m_ros2::srv::Desired>::SharedPtr service_desired_;

  void publishJointState();

  void control_mode_(
    std::shared_ptr<kondo_b3m_ros2::srv::ControlMode::Request> const request,
    std::shared_ptr<kondo_b3m_ros2::srv::ControlMode::Response> response);
  void desired_(
    std::shared_ptr<kondo_b3m_ros2::srv::Desired::Request> const request,
    std::shared_ptr<kondo_b3m_ros2::srv::Desired::Response> response);
  B3mCommand send_command_(B3mCommand const & command);
};

#endif  // KONDO_B3M_HPP_
