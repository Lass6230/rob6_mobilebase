cmake_minimum_required(VERSION 3.8)
project(mobile_base_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(crust_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(main_program src/main_program.cpp)
ament_target_dependencies(
  main_program
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  crust_msgs 
  tf2_geometry_msgs
)

add_executable(joy_listner src/joy_control.cpp)
ament_target_dependencies(
  joy_listner
  rclcpp
  #std_msgs
  sensor_msgs 
)

add_executable(odom_rebroadcaster src/odom_ZOH.cpp)
ament_target_dependencies(
  odom_rebroadcaster
  rclcpp
  tf2
  tf2_ros
  nav_msgs
)


install(TARGETS
  odom_rebroadcaster
  main_program
  joy_listner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  meshes
  urdf
  config
  #controllers
  world
  launch
  maps
  #maps/hall
  #maps/sim
  DESTINATION share/${PROJECT_NAME}
)


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


