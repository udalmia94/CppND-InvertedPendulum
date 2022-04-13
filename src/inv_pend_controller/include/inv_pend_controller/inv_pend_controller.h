#ifndef ROS_INV_PEND_CONTROLLER_H_
#define ROS_INV_PEND_CONTROLLER_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class InvPendulumController : public rclcpp::Node {
public:
  InvPendulumController();
private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_velocity_cmd_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pend_angle_subscription_;

  /// Function that receives the pendulum position and sends velocity commands
  void PendulumPositionUpdateClbk(const sensor_msgs::msg::JointState::SharedPtr msg);

  double pid_p_;
  double pid_d_;
  double pid_i_;

  double integrator_;

  /// Retrieve a ROS parameter associated with this node.
  /// @tparam T The expected type of the parameter
  /// @param param_name The parameter name
  /// @return The parameter value
  template <typename T>
  T GetNodeParam(const char* param_name);
};

template <typename T>
T InvPendulumController::GetNodeParam(const char* param_name) {
  T param{};
  try {
    param = declare_parameter(param_name).get<T>();
  } catch (const rclcpp::ParameterTypeException& ex) {
    RCLCPP_FATAL(
        get_logger(),
        "Failed getting parameter '%s'! Does it exist? Type mismatch maybe?",
        param_name);
    throw;
  }
  return param;
}

#endif
