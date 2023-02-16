cmake_minimum_required(VERSION 3.5)
project(waypoint_navigator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(mav_visualization REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_trajectory_generation_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  mav_msgs
  mav_planning_msgs
  sensor_msgs
  nav_msgs
  tf2
  std_srvs
  visualization_msgs
  mav_visualization
  mav_trajectory_generation
  mav_trajectory_generation_ros
)

set(srv_files
  "srv/ExecutePathFromFile.srv"
  "srv/GoToHeight.srv"
  "srv/GoToPoseWaypoints.srv"
  "srv/GoToWaypoint.srv"
  "srv/GoToWaypoints.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(waypoint_navigator_node 
               src/waypoint_navigator_node.cpp)
ament_target_dependencies(waypoint_navigator_node ${DEPENDENCIES})
target_link_libraries(waypoint_navigator_node glog yaml-cpp)
rosidl_target_interfaces(waypoint_navigator_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS waypoint_navigator_node
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY paths
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY configs
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

ament_export_dependencies(eigen3_cmake_module)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
