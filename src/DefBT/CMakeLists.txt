cmake_minimum_required(VERSION 3.8)
project(DefBT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(behaviortree_cpp REQUIRED)  
find_package(behaviortree_ros2 REQUIRED)  
find_package(nav2_msgs REQUIRED)  
find_package(cust_com REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
add_executable(simple_bt src/simple_bt.cpp)
add_executable(behavior src/behavior.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(simple_bt behaviortree_cpp)
ament_target_dependencies(behavior behaviortree_ros2 nav2_msgs std_msgs cust_com)

install(TARGETS
  talker
  simple_bt
  behavior
  DESTINATION lib/${PROJECT_NAME})


                                 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
