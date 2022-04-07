#include "b3m_publisher.hpp"

B3mPublisher::B3mPublisher() : Node("b3m_publisher"), count_(0)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(500ms, std::bind(&B3mPublisher::timerCallback, this));
}

void B3mPublisher::timerCallback()
{
  auto message = std_msgs::msg::String();
  message.data = "test" + std::to_string(count_++);
  RCLCPP_INFO(get_logger(), "Publishng: %s", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<B3mPublisher>());
  rclcpp::shutdown();
  return 0;
}