cmake_minimum_required(VERSION 3.8)
project(zed_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(std_msgs REQUIRED)

# Include cpp "include" directory
include_directories(include)

# Create cpp executables
add_executable(ROS2_IMU_OpticalFlow_Node src/ROS2_IMU_OpticalFlow_Node.cpp)
ament_target_dependencies(ROS2_IMU_OpticalFlow_Node
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV)

# Install cpp executables
install(TARGETS
  ROS2_IMU_OpticalFlow_Node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Py modules
ament_python_install_package(${PROJECT_NAME})

# Install Py executables
install(PROGRAMS
  zed_sub/visualization.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
