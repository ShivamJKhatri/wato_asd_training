cmake_minimum_required(VERSION 3.10)
project(planner)

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
find_package(nav_msgs REQUIRED)    # OccupancyGrid, Odometry, Path
find_package(geometry_msgs REQUIRED)  # PointStamped goals

# Compiles source files into a library
# A library is not executed, instead other executables can link
# against it to access defined methods and classes.
# We build a library so that the methods defined can be used by
# both the unit test and ROS2 node executables.
add_library(planner_lib
  src/planner_core.cpp)
# Indicate to compiler where to search for header files
target_include_directories(planner_lib
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(planner_lib rclcpp nav_msgs geometry_msgs)

option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright)

  ament_cpplint(FILTERS "-legal/copyright")

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(planner_test test/planner_test.cpp)
  target_link_libraries(planner_test planner_lib)

  install(TARGETS
    planner_test
    DESTINATION lib/${PROJECT_NAME})
endif()

# Create ROS2 node executable from source files
add_executable(planner_node src/planner_node.cpp)
# Link to the previously built library to access planner classes and methods
target_link_libraries(planner_node planner_lib)

# Copy executable to installation location
install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME})

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
