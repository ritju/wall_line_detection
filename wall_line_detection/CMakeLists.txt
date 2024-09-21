cmake_minimum_required(VERSION 3.8)
project(wall_line_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)

nav2_package()

include_directories(
  include
)

add_library(wall_line_detection_lib SHARED src/wall_line_detection.cpp)

set(dependencies
  ament_cmake
  rclcpp
  tf2_geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  capella_ros_msg
  rclcpp_components
  OpenCV
)
target_include_directories(wall_line_detection_lib PUBLIC include)

ament_target_dependencies(wall_line_detection_lib
  ${dependencies}
)

rclcpp_components_register_node(wall_line_detection_lib
    PLUGIN "wall_line_detection_pkg::WallLineDetection"
    EXECUTABLE wall_line_detection)

install(TARGETS
  wall_line_detection_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch param
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(wall_line_detection_lib)
ament_export_dependencies(${dependencies})

ament_package()
