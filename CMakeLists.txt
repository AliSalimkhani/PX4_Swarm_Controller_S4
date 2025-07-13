cmake_minimum_required(VERSION 3.8)
project(px4_swarm_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include headers
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# ───── Executables ──────────────────────────────

add_executable(arming src/Arming.cpp)
ament_target_dependencies(arming rclcpp px4_msgs)

add_executable(weighted_topology_neighbors src/SwarmControllers/WeightedTopology/WeightedTopologyNeighbors.cpp)
ament_target_dependencies(weighted_topology_neighbors
  rclcpp
  px4_msgs
  std_msgs
  custom_msgs
  Eigen3
)

add_executable(weighted_topology_controller src/SwarmControllers/WeightedTopology/WeightedTopologyController.cpp)
ament_target_dependencies(weighted_topology_controller
  rclcpp
  px4_msgs
  std_msgs
  custom_msgs
  Eigen3
)

add_executable(change_waypoint src/ChangeWaypoint.cpp)
ament_target_dependencies(change_waypoint
  rclcpp
  px4_msgs
  std_msgs
)
target_link_libraries(change_waypoint yaml-cpp) # Link yaml-cpp

add_executable(leader_monitor src/LeaderMonitor.cpp)
ament_target_dependencies(leader_monitor
  rclcpp
  px4_msgs
  std_msgs
)

# ───── SwarmAgent Shared Library as Component ─────

add_library(swarm_agent SHARED
  src/SwarmControllers/SwarmAgent.cpp
)

target_include_directories(swarm_agent PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # ADDITION: Include path for nlohmann/json.hpp
  # Assumes json.hpp is located in YOUR_PROJECT_ROOT/include/nlohmann/json.hpp
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann> 
  $<INSTALL_INTERFACE:include/nlohmann>
)

ament_target_dependencies(swarm_agent
  rclcpp
  rclcpp_components
  px4_msgs
  std_msgs
  custom_msgs
  Eigen3
)

target_link_libraries(swarm_agent
  yaml-cpp
)

# Register as component
rclcpp_components_register_nodes(swarm_agent "Controller::SwarmAgent")

# Export component library
ament_export_targets(export_swarm_agent HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components)

# ───── Install Targets ─────

install(TARGETS
  arming
  weighted_topology_neighbors
  weighted_topology_controller
  change_waypoint
  leader_monitor
  swarm_agent
  EXPORT export_swarm_agent
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers (optional but recommended)
install(DIRECTORY include/
  DESTINATION include
)

# ──── Install Python scripts ─────
install(PROGRAMS px4_swarm_controller/simulation_node.py DESTINATION lib/${PROJECT_NAME})

# ───── Launch & Config ─────
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# ───── Testing ─────
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()