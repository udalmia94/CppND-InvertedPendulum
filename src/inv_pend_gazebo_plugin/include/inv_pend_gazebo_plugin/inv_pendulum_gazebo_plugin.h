#include "gazebo_ros/node.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "sensor_msgs/msg/joint_state.hpp"

namespace gazebo
{

class InvPendulumPlugin : public ModelPlugin
{
public: 
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

private: 
  void OnUpdate();
  void WheelVelocityCmdClbk(const sensor_msgs::msg::JointState::SharedPtr msg);

  gazebo_ros::Node::SharedPtr ros_node_;
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection;


  physics::JointPtr pend_joint_;
  physics::JointPtr wheel_joint_right_1_;
  physics::JointPtr wheel_joint_right_2_;
  physics::JointPtr wheel_joint_left_1_;
  physics::JointPtr wheel_joint_left_2_;

  std::vector<physics::JointPtr> wheel_joints_;

};

}
