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

  gazebo_ros::Node::SharedPtr ros_node_;
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

}
