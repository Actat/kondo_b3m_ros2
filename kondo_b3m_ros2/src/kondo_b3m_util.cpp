#include "../include/kondo_b3m_ros2/kondo_b3m_util.hpp"

KondoB3mUtil::KondoB3mUtil()
: Node("kondo_b3m")
{
  this->declare_parameter<int>("publish_frequency", 50);
  this->declare_parameter<std::vector<std::string>>("motor_list", std::vector<std::string>());

  this->get_parameter("publish_frequency", publish_frequency_);

  motor_list_ = std::vector<B3mMotor>{};
  std::vector<std::string> motor_string_list;
  this->get_parameter("motor_list", motor_string_list);
  if (motor_string_list.size() > 0) {
    std::for_each(
      motor_string_list.begin(), motor_string_list.end(),
      [this](std::string motor_string) {
        motor_list_.push_back(B3mMotor(motor_string));
      });
  }

  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::QoS(10));

  client_state_ = this->create_client<kondo_b3m_interfaces::srv::GetState>(
    "/kondo_b3m/get_state");

  client_mode_ = this->create_client<kondo_b3m_interfaces::srv::ControlMode>(
    "/kondo_b3m/control_mode");
  while (!client_mode_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service is not available. Waiting...");
  }
  for (auto & motor : motor_list_) {
    auto req = std::make_shared<kondo_b3m_interfaces::srv::ControlMode::Request>();
    req->id = motor.id();
    req->mode = motor.control_mode();
    auto future_result = client_mode_->async_send_request(req);
  }

  timer_ =
    create_wall_timer(
    std::chrono::nanoseconds((int)(1e9 / publish_frequency_)),
    std::bind(&KondoB3mUtil::cb_timer_, this));
}

void KondoB3mUtil::cb_timer_()
{
  for (auto & motor : motor_list_) {
    auto req = std::make_shared<kondo_b3m_interfaces::srv::GetState::Request>();
    req->id = motor.id();

    auto cb = [this, &motor]
        (rclcpp::Client<kondo_b3m_interfaces::srv::GetState>::SharedFuture res) {
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = rclcpp::Clock().now();
        js.name.push_back(motor.name());
        js.position.push_back(res.get()->position);
        js.velocity.push_back(res.get()->velocity);

        publisher_->publish(js);
      };

    auto future_result = client_state_->async_send_request(req, cb);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KondoB3mUtil>());
  rclcpp::shutdown();
  return 0;
}
