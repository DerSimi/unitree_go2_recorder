set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.16)
project(sim2real_transfer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# wo unitree_sdk2Config.cmake liegt
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

# MuJoCo und Unitree SDK2 holen
find_package(mujoco      REQUIRED)    # liefert mujoco::mujoco
find_package(unitree_sdk2 REQUIRED)    # liefert unitree_sdk2

# alle Demo-Quellen aus dem Original-Repo
file(GLOB SIM_SRC
  ${CMAKE_SOURCE_DIR}/src/mujoco/*.cc
  ${CMAKE_SOURCE_DIR}/src/unitree_sdk2_bridge/*.cc
)

# genau wie im Original-Repo
set(SIM_DEPENDENCIES
  pthread
  mujoco::mujoco
  glfw
  yaml-cpp
  unitree_sdk2
)

# Dein Executable, das alle Demo-Dateien + deine main.cc baut
add_executable(mujoco_extractor
  ${SIM_SRC}
  ${CMAKE_SOURCE_DIR}/src/mujoco_extractor.cpp
)
# include src/ so dass <mujoco/glfw_adapter.h> usw. gefunden wird
target_include_directories(mujoco_extractor
  PRIVATE ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(mujoco_extractor
  PRIVATE ${SIM_DEPENDENCIES}
)

set(CMAKE_BUILD_TYPE Release)