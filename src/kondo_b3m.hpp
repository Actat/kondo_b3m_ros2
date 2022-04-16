#ifndef KONDO_B3M_HPP_
#define KONDO_B3M_HPP_

#include <chrono>
#include <cmath>
#include <vector>
#include "b3m_port.cpp"
#include "kondo_b3m_interfaces/srv/desired_speed.hpp"
#include "kondo_b3m_interfaces/srv/motor_free.hpp"
#include "kondo_b3m_interfaces/srv/start_speed_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class KondoB3m : public rclcpp::Node {
public:
  KondoB3m();

private:
  std::string port_name_;
  B3mPort *port_;
  std::vector<uint8_t> id_list_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Service<kondo_b3m_interfaces::srv::MotorFree>::SharedPtr
      service_free_motor_;
  rclcpp::Service<kondo_b3m_interfaces::srv::StartSpeedControl>::SharedPtr
      service_start_speed_control_;
  rclcpp::Service<kondo_b3m_interfaces::srv::DesiredSpeed>::SharedPtr
      service_desired_speed_;

  void publishJointState();
  void motorFree(
      const std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Request>
          request,
      const std::shared_ptr<kondo_b3m_interfaces::srv::MotorFree::Response>
          response);
  void startSpeedControl(
      const std::shared_ptr<
          kondo_b3m_interfaces::srv::StartSpeedControl::Request> request,
      const std::shared_ptr<
          kondo_b3m_interfaces::srv::StartSpeedControl::Response> response);
  void desiredSpeed(
      const std::shared_ptr<kondo_b3m_interfaces::srv::DesiredSpeed::Request>
          request,
      const std::shared_ptr<kondo_b3m_interfaces::srv::DesiredSpeed::Response>
          response);
  void fillIdList_();
};

#endif  // KONDO_B3M_HPP_
