cmake_minimum_required(VERSION 3.8)
project(simple_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_messages REQUIRED)


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

# Build simple_listener.cpp as listener
add_executable(listener
  src/simple_listener.cpp
)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)

# Build simple_listener_with_custom_message.cpp as listener_with_custom_msg.
add_executable(listener_with_custom_msg
  src/simple_listener_with_custom_message.cpp
)
ament_target_dependencies(listener_with_custom_msg rclcpp std_msgs my_messages)
install(TARGETS
  listener_with_custom_msg
  DESTINATION lib/${PROJECT_NAME}
)

# Copy launch direcotry to install directory,
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
