cmake_minimum_required(VERSION 3.8)
project(go2_unitree_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(UNITREE_SDK2_ROOT "/opt/unitree_sdk2")
set(UNITREE_SDK2_INCLUDE_DIR "${UNITREE_SDK2_ROOT}/include")
set(UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
set(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2_ROOT}/lib/aarch64")
set(UNITREE_SDK2_THIRDPARTY_LIB_DIR "${UNITREE_SDK2_ROOT}/thirdparty/lib/aarch64")

add_executable(go2_unitree_bridge_node src/go2_unitree_bridge_node.cpp)

target_include_directories(go2_unitree_bridge_node PUBLIC
  ${UNITREE_SDK2_INCLUDE_DIR}
  ${UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR}
)

target_link_libraries(go2_unitree_bridge_node
  ${UNITREE_SDK2_LIB_DIR}/libunitree_sdk2.a
  ${UNITREE_SDK2_THIRDPARTY_LIB_DIR}/libddscxx.so
  ${UNITREE_SDK2_THIRDPARTY_LIB_DIR}/libddsc.so
  Threads::Threads
)

find_package(Threads REQUIRED)

install(TARGETS go2_unitree_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
