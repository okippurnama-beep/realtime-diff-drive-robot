cmake_minimum_required(VERSION 3.8)
project(diffbot_navigation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME benchmark_assets_match_layout
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_benchmark_assets.py
      --check
  )

  add_test(
    NAME nav_benchmark_summary_unit
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_nav_benchmark_summary.py
  )

  add_test(
    NAME nav2_params_contract_unit
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_nav2_params.py
  )
endif()

install(
  DIRECTORY config launch maps worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_benchmark_assets.py
    scripts/nav_benchmark_runner.py
    scripts/summarize_nav_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
