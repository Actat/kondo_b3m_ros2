#include "../include/kondo_b3m_ros2/kondo_b3m_util.hpp"

KondoB3mUtil::KondoB3mUtil()
: Node("kondo_b3m_util")
{
  this->declare_parameter<uint8_t>("publish_frequency", 50);
  this->get_parameter<int>("publish_frequency", publish_frequency_);
  this->get_parameter("id", id_);
  this->declare_parameter<uint8_t>("id", 0);
  this->get_parameter("id", id_);
  this->declare_parameter<std::string>("name", "joint");
  this->get_parameter("name", name_);
  this->declare_parameter<std::string>("mode", "position");
  this->get_parameter("mode", mode_);

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
  RCLCPP_INFO(this->get_logger(), "Service /kondo_b3m/control_mode is available.");
  auto req = std::make_shared<kondo_b3m_interfaces::srv::ControlMode::Request>();
  req->id = id_;
  req->mode = mode_;
  auto future_result = client_mode_->async_send_request(req);
  rclcpp::sleep_for(std::chrono::milliseconds(1));

  group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  timer_ =
    create_wall_timer(
    std::chrono::nanoseconds((int)(1e9 / publish_frequency_)),
    std::bind(&KondoB3mUtil::cb_timer_, this), group_);
}

void KondoB3mUtil::cb_timer_()
{
  auto req = std::make_shared<kondo_b3m_interfaces::srv::GetState::Request>();
  req->id = id_;

  auto cb = [this]
      (rclcpp::Client<kondo_b3m_interfaces::srv::GetState>::SharedFuture res) {
      auto js = sensor_msgs::msg::JointState();
      js.header.stamp = rclcpp::Clock().now();
      js.name.push_back(name_);
      js.position.push_back(res.get()->position);
      js.velocity.push_back(res.get()->velocity);

      publisher_->publish(js);
    };

  auto future_result = client_state_->async_send_request(req, cb);
  rclcpp::sleep_for(std::chrono::microseconds(100));

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<KondoB3mUtil>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
