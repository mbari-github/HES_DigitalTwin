cmake_minimum_required(VERSION 3.8)
project(exoskeletron_safety_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# ============================================================
# Libreria plugin
# ============================================================
add_library(safety_plugins SHARED
  src/FaultMonitorMode.cpp
  src/SafeStopMode.cpp
  src/CompliantMode.cpp
  src/TorqueLimitMode.cpp
  src/SensorDegradedMode.cpp
)

ament_target_dependencies(safety_plugins
  rclcpp
  pluginlib
  std_srvs
  std_msgs
)

target_include_directories(safety_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# registra plugins.xml per pluginlib
pluginlib_export_plugin_description_file(
  exoskeletron_safety_manager
  plugins.xml
)

# ============================================================
# Nodo state_machine
# ============================================================
add_executable(state_machine_node
  src/StateMachine.cpp
  src/state_machine_node.cpp
)

ament_target_dependencies(state_machine_node
  rclcpp
  pluginlib
  std_srvs
  std_msgs
)

target_include_directories(state_machine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(state_machine_node
  safety_plugins
)

# ============================================================
# Installazione
# ============================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS safety_plugins state_machine_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(safety_plugins)
ament_export_dependencies(rclcpp pluginlib std_srvs std_msgs)

ament_package()