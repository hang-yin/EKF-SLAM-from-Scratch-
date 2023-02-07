cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(turtlelib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Control.srv" "srv/InitialPose.srv")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(turtle_control src/turtle_interface.cpp)
add_executable(circle src/circle.cpp)
add_executable(odometry src/odometry.cpp)

ament_target_dependencies(turtle_control
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "nuturtlebot_msgs"
  "geometry_msgs"
)

ament_target_dependencies(circle
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "nuturtlebot_msgs"
  "geometry_msgs"
)

ament_target_dependencies(odometry
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "nuturtlebot_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)

# target_link_libraries(nuturtle_interface turtlelib::turtlelib)

target_link_libraries(turtle_control
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${nuturtlebot_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  turtlelib::turtlelib
)

target_link_libraries(circle
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${nuturtlebot_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  turtlelib::turtlelib
  "${cpp_typesupport_target}"
)

target_link_libraries(odometry
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${nuturtlebot_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  turtlelib::turtlelib
  "${cpp_typesupport_target}"
)

install (TARGETS
  turtle_control
  circle
  odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

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
