cmake_minimum_required(VERSION 3.8)
project(astra_gz_sensors)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models worlds config rviz
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
