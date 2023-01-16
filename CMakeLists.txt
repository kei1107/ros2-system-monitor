cmake_minimum_required(VERSION 3.8)
project(ros2_system_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config scripts launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/cpu_monitor_node.py
  scripts/hdd_monitor_node.py
  scripts/mem_monitor_node.py
  scripts/net_monitor_node.py
  scripts/ntp_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
