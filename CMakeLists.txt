cmake_minimum_required(VERSION 3.8)
project(walker)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Set compiler flags for better debugging and code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executables
# add_executable(avoid_obstacle src/avoid_obstacle.cpp)
add_executable(walker src/walker.cpp)

# Link the dependencies to the executables
# ament_target_dependencies(avoid_obstacle rclcpp sensor_msgs geometry_msgs std_msgs)
ament_target_dependencies(walker rclcpp sensor_msgs geometry_msgs std_msgs)

# Install targets
install(TARGETS
  walker
  DESTINATION lib/${PROJECT_NAME}
)

# Install worlds
install(DIRECTORY 
  include/
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Declare the package
ament_package()
