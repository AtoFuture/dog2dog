cmake_minimum_required(VERSION 3.10)
project(traj_drive)

find_package(ignition-cmake2 REQUIRED)
ign_find_package(ignition-gazebo6 REQUIRED
  COMPONENTS core)

add_library(TrajDrive SHARED traj_drive.cc)
set_target_properties(TrajDrive PROPERTIES
  OUTPUT_NAME "TrajDrive")

target_include_directories(TrajDrive PRIVATE
  ${IGNITION-GAZEBO6_INCLUDE_DIRS})
target_link_libraries(TrajDrive
  ${IGNITION-GAZEBO6_LIBRARIES})

install(TARGETS TrajDrive DESTINATION lib)
