set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.16)
project(sim2real_transfer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

# MuJoCo und Unitree SDK2 holen
find_package(mujoco      REQUIRED)  
find_package(unitree_sdk2 REQUIRED)    
find_package(ZLIB REQUIRED)

file(GLOB SIM_SRC
  ${CMAKE_SOURCE_DIR}/src/mujoco/*.cc
  ${CMAKE_SOURCE_DIR}/src/extractor/*.cc
  ${CMAKE_SOURCE_DIR}/src/storage/*.cc
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
)
target_link_libraries(mujoco_extractor
  PRIVATE ${SIM_DEPENDENCIES}
)

# add_executable(circle_walk src/circle_walk.cc)

# target_link_libraries(circle_walk PRIVATE unitree_sdk2)

set(CMAKE_BUILD_TYPE Release)
