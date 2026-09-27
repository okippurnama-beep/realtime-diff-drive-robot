cmake_minimum_required(VERSION 3.8)
project(diffbot_hardware)

find_package(ament_cmake REQUIRED)

add_library(mcu_contract INTERFACE)
target_compile_features(mcu_contract INTERFACE cxx_std_17)
target_include_directories(mcu_contract
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mcu_contract
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(diffbot_interfaces REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mcu_protocol test/test_mcu_protocol.cpp)
  ament_target_dependencies(test_mcu_protocol diffbot_interfaces)
  target_link_libraries(test_mcu_protocol mcu_contract)
endif()

ament_package()
