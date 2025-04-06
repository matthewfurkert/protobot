cmake_minimum_required(VERSION 3.8)
project(protobot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)


install(PROGRAMS
  ${PROJECT_NAME}/send_joint_angles.py
  ${PROJECT_NAME}/inverse_kinematics.py
  ${PROJECT_NAME}/coord_to_joint_angles.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
