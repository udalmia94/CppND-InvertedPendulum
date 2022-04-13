#include "inv_pend_controller/inv_pend_controller.h"

InvPendulumController::InvPendulumController() : Node("inv_pendulum_controller"), integrator_(0) {
  pend_angle_subscription_ = create_subscription<sensor_msgs::msg::JointState>("pendulum_angle", 10, std::bind(&InvPendulumController::PendulumPositionUpdateClbk, this, std::placeholders::_1));

  wheel_velocity_cmd_publisher_ = create_publisher<sensor_msgs::msg::JointState>("wheel_velocity_cmd", 10);

  pid_p_ = GetNodeParam<double>("pid_p");
  pid_i_ = GetNodeParam<double>("pid_i");
  pid_d_ = GetNodeParam<double>("pid_d");
}

void InvPendulumController::PendulumPositionUpdateClbk(sensor_msgs::msg::JointState::SharedPtr msg) {
  auto pub_msg = sensor_msgs::msg::JointState();

  double pendulum_pos = msg->position[0];
  double pendulum_vel = msg->velocity[0];
  integrator_ = integrator_ + pendulum_pos;

  double wheel_vel_cmd = pid_p_ * pendulum_pos + pid_d_ * pendulum_vel + pid_i_ * integrator_;

  pub_msg.velocity = {wheel_vel_cmd, wheel_vel_cmd, wheel_vel_cmd, wheel_vel_cmd};

  wheel_velocity_cmd_publisher_->publish(pub_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InvPendulumController>());
  rclcpp::shutdown();
  return 0;
}
