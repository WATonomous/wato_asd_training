cmake_minimum_required(VERSION 3.10)
project(control)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(sample_msgs REQUIRED) # Custom package containing ROS2 messages
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Compiles source files into a library
add_library(control_lib
  src/control_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(control_lib
  PUBLIC
  include)

# Add ROS2 dependencies required by package
ament_target_dependencies(control_lib
  rclcpp
  sample_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs)

# Create ROS2 node executable from source files
add_executable(control_node src/control_node.cpp)
# Link to the previously built library to access Transformer classes and methods
target_link_libraries(control_node control_lib)
# Ensure control_node knows where to find headers
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Create timer node executable
add_executable(timer_node src/timer_node.cpp)
# Specify dependencies for the timer node
ament_target_dependencies(timer_node
  rclcpp
  std_msgs
  nav_msgs  # Ensure this line is added
  )
# Ensure timer_node knows where to find headers
target_include_directories(timer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Copy executable to installation location
install(TARGETS
  control_node
  timer_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch files to installation location
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
