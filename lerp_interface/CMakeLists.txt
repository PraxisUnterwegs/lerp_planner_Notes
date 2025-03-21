cmake_minimum_required(VERSION 3.22)
project(moveit_planners_lerp)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Set include directories
include_directories(
  include
)

# Add the LERP interface library
add_library(moveit_lerp_planner_plugin SHARED
  src/lerp_interface.cpp
  src/lerp_planner_manager.cpp
  src/lerp_planning_context.cpp
)

# Set target properties
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# Specify the dependencies for the target
ament_target_dependencies(moveit_lerp_planner_plugin
  moveit_core
  moveit_msgs
  trajectory_msgs
  pluginlib
)
target_link_libraries(moveit_lerp_planner_plugin pluginlib::pluginlib)

# Install the library
install(
  TARGETS moveit_lerp_planner_plugin
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the include directory
install(DIRECTORY include/ DESTINATION include)

# Install plugin.xml
pluginlib_export_plugin_description_file(moveit_core lerp_interface_plugin_description.xml)

# Export targets for the package
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core moveit_msgs trajectory_msgs pluginlib)

# Run tests if specified
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
