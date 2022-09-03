cmake_minimum_required(VERSION 3.8)
project(simple_talker)

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

# simple talker
add_executable(talker
  src/simple_talker.cpp
)

ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)

# simple talker with original message
add_executable(talker_with_original_message
  src/simple_talker_with_original_message.cpp
)
ament_target_dependencies(talker_with_original_message rclcpp std_msgs my_messages)
install(TARGETS
  talker_with_original_message
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(talker_with_parameters
  src/simple_talker_with_parameters.cpp
)
ament_target_dependencies(talker_with_parameters rclcpp std_msgs my_messages)
install(TARGETS
  talker_with_parameters
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
