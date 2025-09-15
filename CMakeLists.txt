set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.16)
project(sim2real_transfer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(mujoco      REQUIRED)  
find_package(unitree_sdk2 REQUIRED)    
find_package(ZLIB REQUIRED)

# Ros stuff
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB SIM_SRC
  ${CMAKE_SOURCE_DIR}/src/mujoco/*.cc
  ${CMAKE_SOURCE_DIR}/src/extractor/*.cc
  ${CMAKE_SOURCE_DIR}/src/storage/*.cc
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
  ${CMAKE_SOURCE_DIR}/src/main.cc
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
)

# add_executable(circle_walk src/circle_walk.cc)

# target_link_libraries(circle_walk PRIVATE unitree_sdk2)

set(CMAKE_BUILD_TYPE Release)
