cmake_minimum_required(VERSION 3.8)
project(timed_topics)

# C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Build custom topics
set(msg_files
  "msg/TimedLowState.msg"
  "msg/TimedLowCmd.msg"
  "msg/TimedSportModeState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs unitree_go
)

# Build the republish_node
add_executable(republish_node src/republish_node.cpp)
target_compile_features(republish_node PRIVATE cxx_std_17)

# Include directories
target_include_directories(republish_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(republish_node
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# Link against generated messages in this package
rosidl_target_interfaces(republish_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install rules
install(TARGETS republish_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install python script
install(PROGRAMS
  test_times.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()