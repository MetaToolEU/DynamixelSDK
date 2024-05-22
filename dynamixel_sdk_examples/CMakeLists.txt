cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# read_write_node
# Build
add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

# scan_test
# Build
add_executable(scan_test src/scan_test.cpp)
ament_target_dependencies(scan_test
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)

# static_tf_publisher
# Build
add_executable(static_tf_publisher src/static_tf_publisher.cpp)
ament_target_dependencies(static_tf_publisher
  dynamixel_sdk_custom_interfaces
  tf2_ros
  rclcpp
)

# set_position_publisher
# Build
add_executable(set_position_publisher src/set_position_publisher.cpp)
ament_target_dependencies(set_position_publisher
  dynamixel_sdk_custom_interfaces
  tf2_ros
  rclcpp
)

# Install
install(TARGETS
  read_write_node
  scan_test
  static_tf_publisher
  set_position_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
