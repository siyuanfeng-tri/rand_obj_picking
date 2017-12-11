cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(clutter)
find_package(PCL REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package(realsense)

set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(
  ${CMAKE_INSTALL_PREFIX}/include
  /usr/include/pcl-1.7
  #${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib)

add_definitions(
  -DIIWA_MODEL_PATH="${PROJECT_SOURCE_DIR}/../models/iiwa_description/urdf/iiwa14_polytope_collision.urdf"
  -DGRASP_PARAM_PATH="${PROJECT_SOURCE_DIR}/../cfg/grasp_params")

add_executable(clutter_clearing2 clutter_clearing2.cc)
target_link_libraries(clutter_clearing2 drake::drake perception grasp_gen robot_bridge)
