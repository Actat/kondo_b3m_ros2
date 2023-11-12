#include "../include/kondo_b3m_ros2/kondo_b3m.hpp"

KondoB3m::KondoB3m()
: Node("kondo_b3m")
{
  this->declare_parameter<std::string>("port_name", "/dev/ttyKONDO");
  this->declare_parameter<int>("baudrate", 1500000);
  this->declare_parameter<int>("publish_frequency", 50);
  this->declare_parameter<std::vector<std::string>>("motor_list", std::vector<std::string>());

  this->get_parameter("port_name", port_name_);
  this->get_parameter("baudrate", baudrate_);

  port_ = new B3mPort(port_name_, baudrate_);

  sub_pos_ = this->create_subscription<kondo_b3m_interfaces::msg::SetDesired>(
    "set_position", 10, std::bind(&KondoB3m::set_pos_, this, std::placeholders::_1));
  sub_vel_ = this->create_subscription<kondo_b3m_interfaces::msg::SetDesired>(
    "set_velocity", 10, std::bind(&KondoB3m::set_vel_, this, std::placeholders::_1));

  service_control_mode_ =
    this->create_service<kondo_b3m_interfaces::srv::ControlMode>(
    "~/control_mode",
    std::bind(
      &KondoB3m::control_mode_, this, std::placeholders::_1,
      std::placeholders::_2));
  service_state_ = this->create_service<kondo_b3m_interfaces::srv::GetState>(
    "~/get_state", std::bind(
      &KondoB3m::state_, this, std::placeholders::_1,
      std::placeholders::_2));
}

KondoB3m::~KondoB3m()
{
  B3mCommand command(B3M_COMMAND_WRITE, 0x07, 255,
    std::vector<unsigned char>({0x02, 0x28, 0x01}));
  send_command_(command);
}

// private---------------------------------------------------------------------

void KondoB3m::set_pos_(kondo_b3m_interfaces::msg::SetDesired::SharedPtr const msg)
{
  std::vector<unsigned char> data;

  double deg = msg->value * 360 / (2 * M_PI);
  int16_t cmd = (int16_t)(deg * 100);

  data.push_back(cmd & 0xFF);
  data.push_back((cmd >> 8) & 0xFF);
  data.push_back(0x2A);
  data.push_back(0x01);

  B3mCommand command(B3M_COMMAND_WRITE, 0, msg->id, data);
  auto reply = send_command_(command);
  if (!reply.validated()) {
    RCLCPP_WARN(this->get_logger(), "Desired value is not set due to write failure.");
  }
}

void KondoB3m::set_vel_(kondo_b3m_interfaces::msg::SetDesired::SharedPtr const msg)
{
  std::vector<unsigned char> data;

  double deg_s = msg->value * 360 / 2 / M_PI;
  int16_t cmd = (int16_t)(deg_s * 100);

  data.push_back(cmd & 0xFF);
  data.push_back((cmd >> 8) & 0xFF);
  data.push_back(0x30);
  data.push_back(0x01);

  B3mCommand command(B3M_COMMAND_WRITE, 0, msg->id, data);
  auto reply = send_command_(command);
  if (!reply.validated()) {
    RCLCPP_WARN(this->get_logger(), "Desired value is not set due to write failure.");
  }
}

void KondoB3m::control_mode_(
  std::shared_ptr<kondo_b3m_interfaces::srv::ControlMode::Request> const request,
  std::shared_ptr<kondo_b3m_interfaces::srv::ControlMode::Response> response)
{
  response->success = true;

  auto const mode = request->mode;
  unsigned char mode_byte, gain_byte;    // mode: 0x28, gain: 0x5c
  if (mode == "free" || mode == "fre" || mode == "f") {
    mode_byte = B3M_MOTOR_MODE_F;
  } else if (mode == "position" || mode == "pos" || mode == "p") {
    mode_byte = B3M_MOTOR_MODE_P;
    gain_byte = 0;
  } else if (mode == "speed" || mode == "spd" || mode == "s") {
    mode_byte = B3M_MOTOR_MODE_S;
    gain_byte = 1;
  } else if (mode == "torque" || mode == "trq" || mode == "t") {
    mode_byte = B3M_MOTOR_MODE_T;
    gain_byte = 2;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      ("Control mode '" + mode + "' is not recognized.").c_str());
    response->success = false;
    return;
  }

  auto const id = request->id;

  B3mCommand cmd_free(B3M_COMMAND_WRITE, 0, id,
    std::vector<unsigned char>({B3M_MOTOR_MODE_F, 0x28, 0x01}));
  auto reply_free = send_command_(cmd_free);
  if (!reply_free.validated()) {
    RCLCPP_WARN(this->get_logger(), "Motor is not free due to write failure.");
    response->success = false;
    return;
  }

  B3mCommand cmd_gain(B3M_COMMAND_WRITE, 0, id,
    std::vector<unsigned char>({gain_byte, 0x5c, 0x01}));
  auto reply_gain = send_command_(cmd_gain);
  if (!reply_gain.validated()) {
    RCLCPP_WARN(this->get_logger(), "Gain is not set due to write failure.");
    response->success = false;
    return;
  }

  B3mCommand cmd_mode(B3M_COMMAND_WRITE, 0, id,
    std::vector<unsigned char>({mode_byte, 0x28, 0x01}));
  auto reply_mode = send_command_(cmd_mode);
  if (!reply_mode.validated()) {
    RCLCPP_WARN(this->get_logger(), "Mode is not set due to write failure.");
    response->success = false;
    return;
  }
}

void KondoB3m::state_(
  std::shared_ptr<kondo_b3m_interfaces::srv::GetState::Request> const request,
  std::shared_ptr<kondo_b3m_interfaces::srv::GetState::Response> response)
{
  B3mCommand command(B3M_COMMAND_READ, 0x00, request->id,
    std::vector<unsigned char>({0x2A, 0x20}));
  auto reply = this->send_command_(command);
  if (!reply.validated()) {
    return;
  }

  int16_t p = reply.data().at(3) << 8 | reply.data().at(2);
  int16_t v = reply.data().at(9) << 8 | reply.data().at(8);

  response->position = std::fmod(M_PI * p / 18000 + M_PI, 2 * M_PI) - M_PI;
  response->velocity = M_PI * v / 18000;
  return;
}

B3mCommand KondoB3m::send_command_(B3mCommand const & command)
{
  if (!port_->write_device(command)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send command.");
    return B3mCommand();
  }
  if (!command.expect_reply()) {
    B3mCommand cmd;
    cmd.set_validated();
    return cmd;
  }

  auto read = port_->read_device();
  if (!read.validated()) {
    RCLCPP_WARN(this->get_logger(), "Failed to read command.");
    return read;
  }

  return read;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KondoB3m>());
  rclcpp::shutdown();
  return 0;
}
