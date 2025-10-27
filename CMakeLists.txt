set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.16)
project(sim2real_transfer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mujoco      REQUIRED)     
find_package(ZLIB REQUIRED)

find_package(spdlog REQUIRED)
find_package(vicon-datastream-sdk REQUIRED)

# Ros stuff
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB SIM_SRC
  ${CMAKE_SOURCE_DIR}/src/mujoco/*.cc
  ${CMAKE_SOURCE_DIR}/src/extractor/*.cpp
  ${CMAKE_SOURCE_DIR}/src/storage/*.cpp
  
  ${CMAKE_SOURCE_DIR}/src/base/*.cpp
  ${CMAKE_SOURCE_DIR}/src/base/low_state/*.cpp
  ${CMAKE_SOURCE_DIR}/src/base/low_cmd/*.cpp
  ${CMAKE_SOURCE_DIR}/src/base/high_state/*.cpp
  ${CMAKE_SOURCE_DIR}/src/base/go2_odometry/*.cpp
  ${CMAKE_SOURCE_DIR}/src/base/vicon/*.cpp
)

set(SIM_DEPENDENCIES
  pthread
  mujoco::mujoco
  glfw
  unitree_sdk2
  ZLIB::ZLIB
  cnpy
)

add_executable(mujoco_extractor
  ${SIM_SRC}
  ${CMAKE_SOURCE_DIR}/src/main.cpp
)

target_include_directories(mujoco_extractor
  PRIVATE ${CMAKE_SOURCE_DIR}/src
  PRIVATE /opt/ros/${ROS_DISTRO}/include
  PRIVATE ${tf2_msgs_INCLUDE_DIRS}
  PRIVATE ${unitree_go_INCLUDE_DIRS}
  PRIVATE ${nav_msgs_INCLUDE_DIRS}
  PRIVATE ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(mujoco_extractor
  PRIVATE ${SIM_DEPENDENCIES}
  PRIVATE rclcpp::rclcpp
  PRIVATE ${tf2_msgs_LIBRARIES}
  PRIVATE ${unitree_go_LIBRARIES}
  PRIVATE ${nav_msgs_LIBRARIES}
  PRIVATE Eigen3::Eigen
  PRIVATE vicon-datastream-sdk::ViconDataStreamSDK_CPP
  PRIVATE spdlog::spdlog
)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
