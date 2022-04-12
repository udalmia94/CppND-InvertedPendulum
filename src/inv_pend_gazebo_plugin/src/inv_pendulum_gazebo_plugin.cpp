#include "inv_pend_gazebo_plugin/inv_pendulum_gazebo_plugin.h"

namespace gazebo
{

void InvPendulumPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  ros_node_ = gazebo_ros::Node::Get(sdf);

  model_ = parent;

  publisher_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("pendulum_angle", 10);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&InvPendulumPlugin::OnUpdate, this));
}

void InvPendulumPlugin::OnUpdate() {
  auto message = sensor_msgs::msg::JointState();

  auto pend_joint = model_->GetJoint("pendulum_hinge");

  message.position = {pend_joint->Position(0)};

  publisher_->publish(message);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(InvPendulumPlugin)

}
