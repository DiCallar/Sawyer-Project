cmake_minimum_required(VERSION 3.0.2)
project(cu_sawyer_gamepad)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  control_msgs
)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ControllerToArm src/ControllerToArm.cpp)

target_link_libraries(ControllerToArm ${catkin_LIBRARIES})
