################################################################################
# PROJECT SETUP
################################################################################
cmake_minimum_required(VERSION 3.8)
project(motion_planner)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# DEPENDENCIES
################################################################################

# Core ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# MoveIt2
find_package(moveit_core REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# MoveIt Task Constructor (MUST be before message generation)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

# TF2
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Message generation
find_package(rosidl_default_generators REQUIRED)

################################################################################
# MESSAGE GENERATION
################################################################################

# Generate custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteTask.srv"
  "srv/ExecuteAllTasks.srv"
  DEPENDENCIES geometry_msgs
)

################################################################################
# EXECUTABLES AND LIBRARIES
################################################################################

# Task stages library with planners and task execution
add_library(mtc_task_stages_lib src/mtc_task_stages.cpp)

# Task node library with services
add_library(mtc_task_node_lib src/mtc_task_node.cpp)

# Main entry point executable
add_executable(mtc_main src/mtc_main.cpp)

################################################################################
# INCLUDE DIRECTORIES
################################################################################

# mtc_task_stages_lib library
target_include_directories(mtc_task_stages_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# mtc_task_node_lib library
target_include_directories(mtc_task_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# mtc_main executable
target_include_directories(mtc_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

################################################################################
# LIBRARY AND EXECUTABLE DEPENDENCIES
################################################################################

# mtc_task_stages_lib: Task stages with planners
ament_target_dependencies(mtc_task_stages_lib
  rclcpp
  moveit_core
  moveit_planners_ompl
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  geometry_msgs
)

# mtc_task_node_lib: Task node with services
ament_target_dependencies(mtc_task_node_lib
  rclcpp
  std_srvs
  moveit_core
  moveit_planners_ompl
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

# mtc_main: Main program
ament_target_dependencies(mtc_main
  rclcpp
  moveit_core
  moveit_planners_ompl
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  geometry_msgs
)

################################################################################
# MESSAGE INTERFACES
################################################################################

# Get typesupport target for service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link service interfaces and libraries
target_link_libraries(mtc_task_node_lib "${cpp_typesupport_target}")
target_link_libraries(mtc_main "${cpp_typesupport_target}" mtc_task_node_lib mtc_task_stages_lib)

################################################################################
# INSTALLATION
################################################################################

# Install executables
install(TARGETS
  mtc_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  mtc_task_node_lib
  mtc_task_stages_lib
  DESTINATION lib
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install srdf files
install(DIRECTORY srdf/
  DESTINATION share/${PROJECT_NAME}/srdf
)

# Install urdf files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install include directory
install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# TESTING
################################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check - uncomment when copyright added to all files
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint - uncomment when in git repo with copyright in all files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# PACKAGE SETUP
################################################################################

ament_package()
