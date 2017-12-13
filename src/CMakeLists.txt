cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(clutter)
find_package(drake CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(realsense)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

set(CMAKE_VERBOSE_MAKEFILE ON)
# to preserve path for the exectuable after install
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
  ${CMAKE_INSTALL_PREFIX}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib)

add_definitions(
  -DIIWA_MODEL_PATH="${PROJECT_SOURCE_DIR}/../models/iiwa_description/urdf/iiwa14_polytope_collision.urdf"
  -DGRASP_PARAM_PATH="${PROJECT_SOURCE_DIR}/../cfg/grasp_params")

#add_executable(clutter_clearing2 clutter_clearing2.cc)
#target_link_libraries(clutter_clearing2 perception grasp_gen robot_bridge drake::drake)

add_executable(test_robot_bridge test_robot_bridge.cc)
target_link_libraries(test_robot_bridge perception grasp_gen robot_bridge drake::drake ${VTK_LIBRARIES})

install(TARGETS test_robot_bridge
 DESTINATION ${CMAKE_INSTALL_PREFIX})
