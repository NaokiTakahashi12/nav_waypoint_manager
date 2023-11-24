cmake_minimum_required(VERSION 3.8)
project(nav_waypoint_server
  VERSION 0.0.0
  LANGUAGES
    CXX
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/nav_waypoint_server_node.cpp
  src/nav_route_server_node.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "nav_waypoint_server::NavWaypointServerNode"
  EXECUTABLE nav_waypoint_server_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "nav_waypoint_server::NavRouteServer"
  EXECUTABLE nav_route_server_node
)
generate_parameter_library(nav_waypoint_server_node_params
  src/nav_waypoint_server_node_params.yaml
)
generate_parameter_library(nav_route_server_node_params
  src/nav_route_server_node_params.yaml
)
target_link_libraries(${PROJECT_NAME}
  nav_waypoint_server_node_params
  nav_route_server_node_params
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
