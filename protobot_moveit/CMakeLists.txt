cmake_minimum_required(VERSION 3.8)
project(protobot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(simple_moveit_interface src/simple_moveit_interface.cpp)
ament_target_dependencies(simple_moveit_interface rclcpp moveit_ros_planning_interface)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS simple_moveit_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
