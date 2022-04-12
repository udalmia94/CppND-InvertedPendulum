cmake_minimum_required(VERSION 3.5)
project(inv_pend_gazebo_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(inv_pendulum_sim_plugin SHARED src/inv_pendulum_gazebo_plugin.cpp)
ament_target_dependencies(inv_pendulum_sim_plugin rclcpp gazebo_dev sensor_msgs gazebo_ros) 
# target_link_libraries(inv_pendulum_sim_plugin ${GAZEBO_LIBRARIES} gazebo_ros rclcpp sensor_msgs)
target_include_directories(inv_pendulum_sim_plugin PUBLIC include)

ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

ament_package()
