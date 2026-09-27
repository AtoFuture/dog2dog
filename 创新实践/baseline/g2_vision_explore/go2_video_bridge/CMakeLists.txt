cmake_minimum_required(VERSION 3.8)
project(go2_video_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)

# The ROS Humble development image already provides these FFmpeg libraries and
# headers.  Keep this check explicit so a missing -dev environment fails at
# configure time instead of producing a half-buildable bridge.
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
  libavcodec
  libavutil
  libswscale
)

add_library(h264_stream_decoder
  src/h264_stream_decoder.cpp
)
target_include_directories(h264_stream_decoder
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(h264_stream_decoder PUBLIC cxx_std_17)
target_link_libraries(h264_stream_decoder PUBLIC PkgConfig::FFMPEG)

add_executable(go2_video_bridge_node
  src/go2_video_bridge_node.cpp
)
target_compile_features(go2_video_bridge_node PUBLIC cxx_std_17)
target_link_libraries(go2_video_bridge_node h264_stream_decoder)
ament_target_dependencies(go2_video_bridge_node
  rclcpp
  sensor_msgs
  unitree_go
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS h264_stream_decoder go2_video_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

include(CTest)
set(GO2_BRIDGE_TEST_FILE "" CACHE FILEPATH
  "Annex-B H.264 file used by the streaming chunk test")

if(BUILD_TESTING)
  add_executable(h264_chunk_test
    test/h264_chunk_test.cpp
  )
  target_compile_features(h264_chunk_test PUBLIC cxx_std_17)
  target_link_libraries(h264_chunk_test PRIVATE h264_stream_decoder)

  if(GO2_BRIDGE_TEST_FILE)
    add_test(
      NAME h264_streaming_chunk_boundaries
      COMMAND h264_chunk_test "${GO2_BRIDGE_TEST_FILE}"
    )
  endif()
endif()

ament_package()
