cmake_minimum_required(VERSION 3.8)
project(diffbot_safety)

find_package(ament_cmake REQUIRED)

add_library(safety_policy src/safety_policy.cpp)
target_compile_features(safety_policy PUBLIC cxx_std_17)
target_include_directories(safety_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS safety_policy
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_policy test/test_safety_policy.cpp)
  target_link_libraries(test_safety_policy safety_policy)
endif()

ament_package()
