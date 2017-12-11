cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(rand_obj_picking)
set(drake_DIR "$ENV{HOME}/drake_bazel_build/lib/cmake/drake")

include(ExternalProject)

find_package(PCL REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(realsense)

ExternalProject_Add(robot_bridge_proj
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/robot_bridge
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/build
  -Ddrake_DIR=${drake_DIR})
ExternalProject_Add(rgbd_bridge_proj
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/rgbd_bridge
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/build)
ExternalProject_Add(perception_proj
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/perception
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/build
  -Ddrake_DIR=${drake_DIR})
ExternalProject_Add(grasp_gen_proj
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/grasp_gen
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/build)

ExternalProject_Add(clutter_proj
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/src
  DEPENDS robot_bridge_proj rgbd_bridge_proj perception_proj grasp_gen_proj
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}/build
  -Ddrake_DIR=${drake_DIR})

#include_directories(
#  ${PROJECT_SOURCE_DIR}/build/include
#  ${PCL_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS})

#add_executable(clutter_clearing2 clutter_clearing2.cc)
#add_dependencies(clutter_clearing2 robot_bridge_proj rgbd_bridge_proj perception_proj grasp_gen_proj)
#target_link_libraries(clutter_clearing2 perception grasp_gen robot_bridge)
