cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(clutter)
find_package(drake CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
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
  -DMODEL_DIR="${PROJECT_SOURCE_DIR}/../models/"
  -DCONFIG_DIR="${PROJECT_SOURCE_DIR}/../cfg/")

add_executable(clutter_clearing2 clutter_clearing2.cc)
target_link_libraries(clutter_clearing2
  perception
  grasp_gen
  robot_bridge
  rgbd_bridge
  drake::drake
  # idk why i need to do these two explicitly again?? vtk should be covered by
  # drake, and pcl stuff are linked in perception, grasp_gen, rgbd_bridge.
  ${PCL_LIBRARIES})

add_executable(test_robot_bridge test_robot_bridge.cc)
target_link_libraries(test_robot_bridge
  perception
  robot_bridge
  drake::drake)

add_executable(test_plate test_plate.cc)
target_link_libraries(test_plate
  perception
  robot_bridge
  drake::drake)

add_executable(test_cup_flip test_cup_flip.cc util.cc)
target_link_libraries(test_cup_flip
  perception
  robot_bridge
  rgbd_bridge
  drake::drake
  ${PCL_LIBRARIES})

add_executable(test_scan test_scan.cc util.cc)
target_link_libraries(test_scan
  perception
  robot_bridge
  rgbd_bridge
  drake::drake
  ${PCL_LIBRARIES})

add_executable(test_fit test_fit.cc util.cc)
target_link_libraries(test_fit
  perception
  robot_bridge
  rgbd_bridge
  drake::drake
  ${PCL_LIBRARIES})

add_executable(test_cluster test_cluster.cc)
target_link_libraries(test_cluster
  perception
  robot_bridge
  rgbd_bridge
  drake::drake
  ${PCL_LIBRARIES})

add_executable(test_grasp test_grasp.cc)
target_link_libraries(test_grasp grasp_gen perception drake::drake ${PCL_LIBRARIES})

install(TARGETS test_robot_bridge
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS test_plate
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS test_cup_flip
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS test_scan
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS test_fit
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS test_cluster
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS test_grasp
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(TARGETS clutter_clearing2
  DESTINATION ${CMAKE_INSTALL_PREFIX})
