cmake_minimum_required(VERSION 3.16)
project(astra_px4_offboard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(astra_offboard_node src/astra_offboard_node.cpp)
ament_target_dependencies(astra_offboard_node rclcpp px4_msgs)

install(TARGETS astra_offboard_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/preview_bspline.py scripts/audit_frame_alignment.py
  scripts/guarded_bspline_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
