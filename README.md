# CppND-InvertedPendulum

This project is a simulation of an inverted pendulum on a cart. The simulation is performed using Gazebo and the PID control is performed in ROS2.

## Installation Instruction

Install ROS2, Gazebo and gazebo_ros_pkgs, instructions found here: [Installing gazebo_ros_pkgs (ROS 2)](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros).

To build the project packages, in the base folder for this project, run: `colcon build`

## Run the Simulation

### Launch Inverted Pendulum Controller ROS2 node

```
source ./install/local_setup.bash
ros2 run inv_pend_controller inv_pend_controller --ros-args --params-file ./config/pid_params_1.yaml
```

The controller node takes parameters to set the gains for the PID controllers. The gains can be specified in a YAML file. Four YAML files with different PID gains can be found in the `config` folder with different gains. Feel free to try the different files or specify different gains to see how the simulation works.

### Launch the Gazebo Inverted Pendulum simulation

In a different terminal window execute:

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/CppND-InvertedPendulum/build/inv_pend_gazebo_plugin/
gazebo ./inverted_pendulum.world
```

Make sure to specify the correct path to the project directory, if it's not saved in your `$HOME` directory. 

## Results

When running the simulation, you can see a cart with an inverted pendulum on it. The Controller node takes angular position and velocity measurements from the Gazebo simulation node, using the ROS2 pub-sub system. The Controller node publishes commands in the form of wheel velocities that are read and executed by the Gazebo plugin. The Gazebo plugin also has a timed disturbance that's applied to the pendulum every 10 seconds, to show the controller in action.

The four parameter files in the config folder show different levels of control. The first set of parameters in `pid_params_1.yaml` has a really high P gain, and no D or I gain. This is meant to show the controller in action, This is included mainly since the better controllers barely show any movement.

The next set of parameters in `pid_params_2.yaml` has a better performing P controller. `pid_params_3.yaml` adds a D controller, and finally `pid_params_4.yaml` adds an I controller as well.

There is room for improvement in the gain tuning, but the current results show the successful implementation of an inverted pendulum simulation using Gazebo and ROS.

## Structure

### `InvPendulumController`

The class `InvPendulumController` is part of the `inv_pend_controller` ROS2 package. This class implements a subscription `pendulum_angle` which takes info from the Gazebo plugin. This info is used by the PID controller to command wheel velocities for the cart, which are published through `wheel_velocity_cmd` topic.

The class has member variables storing the PID controller gains which are taken as ROS2 parameters and a function which is called every time a message is received on the topic `pendulum_angle`.

### `InvPendulumPlugin`

This class is part of the `inv_pendulum_gazebo_plugin` package and implements a Gazebo plugin. It uses `gazebo_ros` to run a ROS2 node which publishes and subscribes to the message interface described in `InvPendulumController`.

Member functions in this class are `Load` which runs when the sim is loaded and `OnUpdate` which is called on every update of the simulation. These are standard functions in a Gazebo plugin. Another member function is `AddDisturbace`, which is called periodically to add a disturbance to the pendulum, and thus show the controller in action. Finally there's `WheelVelocityCmdClbk` which takes commands from the controller and applies the velocity to the wheels.

Member variables for this class are the joints on the cart, and other member variables to implement the ROS2 and Gazebo functionality described here.

## Rubric Items

- The project demonstrates an understanding of C++ functions and control structures.
  - A variety of control structures are used in the project and the project code is clearly organized into functions.
- The project reads data from a file and process the data, or the program writes data to a file.
  - The `InvPendulumController` class uses ROS2 parameters that are read from a YAML file.
- Classes use appropriate access specifiers for class members.
- Class constructors utilize member initialization lists.
  - `InvPendulumController` uses an initialization list. 
- Templates generalize functions in the project.
  - The `GetNodeParam` function in the `InvPendulumController` class uses templates.
- The project uses smart pointers instead of raw pointers.

## Reference

Since this project focussed on C++, the Gazebo model for this project was taken from: <https://github.com/rbv188/Inverted-Pendulum-Simulation/blob/master/worlds/inverted_pendulum.world>.
