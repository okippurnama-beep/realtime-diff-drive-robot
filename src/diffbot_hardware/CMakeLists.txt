cmake_minimum_required(VERSION 3.8)
project(diffbot_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mcu_contract INTERFACE)
target_compile_features(mcu_contract INTERFACE cxx_std_17)
target_include_directories(mcu_contract
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(fake_mcu
  src/fake_mcu.cpp
  src/fake_mcu_transport.cpp
)
set_target_properties(fake_mcu PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(fake_mcu PUBLIC cxx_std_17)
target_include_directories(fake_mcu
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fake_mcu PUBLIC mcu_contract)

add_library(diffbot_system_hardware SHARED
  src/diffbot_system_hardware.cpp
)
target_compile_features(diffbot_system_hardware PUBLIC cxx_std_17)
target_include_directories(diffbot_system_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diffbot_system_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(diffbot_system_hardware fake_mcu)

pluginlib_export_plugin_description_file(
  hardware_interface diffbot_hardware_plugins.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mcu_contract fake_mcu diffbot_system_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(diffbot_interfaces REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mcu_protocol test/test_mcu_protocol.cpp)
  ament_target_dependencies(test_mcu_protocol diffbot_interfaces)
  target_link_libraries(test_mcu_protocol mcu_contract)

  ament_add_gtest(test_fake_mcu test/test_fake_mcu.cpp)
  target_link_libraries(test_fake_mcu fake_mcu)

  ament_add_gtest(test_fake_mcu_transport test/test_fake_mcu_transport.cpp)
  target_link_libraries(test_fake_mcu_transport fake_mcu)

  ament_add_gtest(test_diffbot_system_hardware test/test_diffbot_system_hardware.cpp)
  ament_target_dependencies(test_diffbot_system_hardware
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(test_diffbot_system_hardware diffbot_system_hardware)
endif()

ament_package()
