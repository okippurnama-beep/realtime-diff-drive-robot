cmake_minimum_required(VERSION 3.8)
project(diffbot_safety)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diffbot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(safety_policy src/safety_policy.cpp)
target_compile_features(safety_policy PUBLIC cxx_std_17)
target_include_directories(safety_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(safety_supervisor_node_lib src/safety_supervisor_node.cpp)
target_compile_features(safety_supervisor_node_lib PUBLIC cxx_std_17)
target_include_directories(safety_supervisor_node_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_supervisor_node_lib
  diagnostic_msgs
  diffbot_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(safety_supervisor_node_lib safety_policy)

add_executable(safety_supervisor_node src/safety_supervisor_main.cpp)
target_compile_features(safety_supervisor_node PUBLIC cxx_std_17)
ament_target_dependencies(safety_supervisor_node rclcpp)
target_link_libraries(safety_supervisor_node safety_supervisor_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    safety_policy
    safety_supervisor_node_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS safety_supervisor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/verify_runtime_command_chain.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_msgs
  diffbot_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_policy test/test_safety_policy.cpp)
  target_link_libraries(test_safety_policy safety_policy)

  ament_add_gtest(
    test_safety_supervisor_node
    test/test_safety_supervisor_node.cpp
    TIMEOUT 30
  )
  ament_target_dependencies(test_safety_supervisor_node
    diagnostic_msgs
    diffbot_interfaces
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    std_srvs
  )
  target_link_libraries(
    test_safety_supervisor_node
    safety_supervisor_node_lib
  )

  ament_add_gtest(
    test_safety_watchdogs
    test/test_safety_watchdogs.cpp
    TIMEOUT 60
  )
  ament_target_dependencies(test_safety_watchdogs
    diagnostic_msgs
    diffbot_interfaces
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    std_srvs
  )
  target_link_libraries(
    test_safety_watchdogs
    safety_supervisor_node_lib
  )

  add_test(
    NAME command_chain_contract_unit
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_command_chain_contract.py
  )
endif()

ament_package()
