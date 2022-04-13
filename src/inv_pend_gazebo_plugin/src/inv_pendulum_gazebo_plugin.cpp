#include "inv_pend_gazebo_plugin/inv_pendulum_gazebo_plugin.h"

namespace gazebo
{

void InvPendulumPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  ros_node_ = gazebo_ros::Node::Get(sdf);

  model_ = parent;

  pend_joint_ = model_->GetJoint("pendulum_hinge");
  wheel_joint_right_1_ = model_->GetJoint("right_wheel_hinge");
  wheel_joint_right_2_ = model_->GetJoint("right_wheel_hinge_2");
  wheel_joint_left_1_ = model_->GetJoint("left_wheel_hinge");
  wheel_joint_left_2_ = model_->GetJoint("left_wheel_hinge_2");

  wheel_joints_ = {wheel_joint_right_1_ ,wheel_joint_right_2_ ,wheel_joint_left_1_ ,wheel_joint_left_2_};

  pend_angle_publisher_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("pendulum_angle", 10);

  wheel_velocity_cmd_subscribtion_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>("wheel_velocity_cmd", 10, 
      std::bind(&InvPendulumPlugin::WheelVelocityCmdClbk, this, std::placeholders::_1));

  add_disturbance_timer_ = ros_node_->create_wall_timer(std::chrono::duration<double>(1.0 / kDisturanceFreqHz), std::bind(&InvPendulumPlugin::AddDisturbance, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&InvPendulumPlugin::OnUpdate, this));
}

void InvPendulumPlugin::OnUpdate() {
  auto message = sensor_msgs::msg::JointState();

  message.position = {pend_joint_->Position(0)};
  message.velocity = {pend_joint_->GetVelocity(0)};

  pend_angle_publisher_->publish(message);
}

void InvPendulumPlugin::WheelVelocityCmdClbk(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::vector<double> velocities = msg->velocity;

  for (int i = 0; i < 4; i++) {
    wheel_joints_[i]->SetVelocity(0, velocities[i]);
  }
}

void InvPendulumPlugin::AddDisturbance() {
  pend_joint_->SetVelocity(0, 0.02);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(InvPendulumPlugin)

}
