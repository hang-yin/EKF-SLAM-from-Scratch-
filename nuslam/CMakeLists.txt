cmake_minimum_required(VERSION 3.8)
project(nuslam)

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
find_package(visualization_msgs REQUIRED)
find_package(Armadillo)

add_executable(slam src/slam.cpp)
add_executable(landmarks src/landmarks.cpp)
add_executable(uda_slam src/uda_slam.cpp)
add_executable(turtle_slam src/turtle_slam.cpp)
add_executable(landmarks_uda src/landmarks_uda.cpp)

ament_target_dependencies(slam
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  turtlelib
  visualization_msgs
)

ament_target_dependencies(landmarks
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  turtlelib
  visualization_msgs
)

ament_target_dependencies(landmarks_uda
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  turtlelib
  visualization_msgs
)

ament_target_dependencies(uda_slam
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  turtlelib
  visualization_msgs
)

ament_target_dependencies(turtle_slam
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  nuturtlebot_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  turtlelib
  visualization_msgs
)

target_link_libraries(slam turtlelib::turtlelib)
target_link_libraries(slam ${ARMADILLO_LIBRARIES})

target_link_libraries(landmarks turtlelib::turtlelib)
target_link_libraries(landmarks ${ARMADILLO_LIBRARIES})

target_link_libraries(landmarks_uda turtlelib::turtlelib)
target_link_libraries(landmarks_uda ${ARMADILLO_LIBRARIES})

target_link_libraries(uda_slam turtlelib::turtlelib)
target_link_libraries(uda_slam ${ARMADILLO_LIBRARIES})

target_link_libraries(turtle_slam turtlelib::turtlelib)
target_link_libraries(turtle_slam ${ARMADILLO_LIBRARIES})

install(TARGETS
  slam
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  landmarks
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  landmarks_uda
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  uda_slam
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  turtle_slam
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/nuslam
)

install(DIRECTORY
  config
  DESTINATION share/nuslam
)

install(DIRECTORY
  src
  DESTINATION share/nuslam
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
