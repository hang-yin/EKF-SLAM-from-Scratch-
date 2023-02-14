cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv "srv/Teleport.srv" LIBRARY_NAME ${PROJECT_NAME} DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(nusim_node src/nusim.cpp)

ament_target_dependencies(nusim_node
                          rclcpp
                          std_msgs
                          std_srvs
                          geometry_msgs
                          tf2
                          tf2_ros
                          visualization_msgs
                          nuturtlebot_msgs
                          sensor_msgs
                          nav_msgs)

target_link_libraries(nusim_node turtlelib::turtlelib)
target_link_libraries(nusim_node "${cpp_typesupport_target}")

target_link_libraries(nusim_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${nuturtlebot_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
)

install(TARGETS
  nusim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/nusim
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
