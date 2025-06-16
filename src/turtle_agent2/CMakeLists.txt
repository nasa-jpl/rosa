cmake_minimum_required(VERSION 3.8)
project(turtle_agent2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables and supporting modules
install(PROGRAMS
  scripts/turtle_agent2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts and modules to lib directory for executable access
install(FILES
  scripts/help.py
  scripts/llm.py
  scripts/prompts.py
  scripts/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install tools package
install(DIRECTORY
  scripts/tools/
  DESTINATION lib/${PROJECT_NAME}/tools
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Ensure XML launch files are executable
# install(FILES
#   launch/agent.launch.xml
#   DESTINATION share/${PROJECT_NAME}/launch
# )

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
