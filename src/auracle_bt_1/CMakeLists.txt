cmake_minimum_required(VERSION 3.8)
project(auracle_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(bt_runner src/bt_runner.cpp)
target_compile_features(bt_runner PUBLIC cxx_std_17)
ament_target_dependencies(bt_runner
  rclcpp rclcpp_action std_msgs geometry_msgs nav2_msgs ament_index_cpp)
target_link_libraries(bt_runner BT::behaviortree_cpp yaml-cpp)

install(TARGETS bt_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()