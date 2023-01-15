cmake_minimum_required(VERSION 3.8)
project(ros2_system_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/cpu_monitor.py
  ${PROJECT_NAME}/mem_monitor.py
  ${PROJECT_NAME}/net_monitor.py
  ${PROJECT_NAME}/ntp_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
