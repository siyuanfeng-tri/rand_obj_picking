cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(rand_obj_picking)
set(drake_DIR "$ENV{HOME}/drake_bazel_build/lib/cmake/drake")

find_package(PCL REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(realsense)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/build)

add_subdirectory(robot_bridge)
add_subdirectory(rgbd_bridge)
add_subdirectory(perception)
add_subdirectory(grasp_gen)

include_directories(
  ${CMAKE_INSTALL_PREFIX}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_executable(clutter_clearing2 clutter_clearing2.cc)
target_link_libraries(clutter_clearing2 perception grasp_gen robot_bridge)
