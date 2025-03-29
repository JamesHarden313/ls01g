cmake_minimum_required(VERSION 3.8)
project(ls01g)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(ls01g src/main.cpp)  # Adjust source file name as needed

# Link libraries
ament_target_dependencies(ls01g
  rclcpp
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  ls01g
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install include files (if any)
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
