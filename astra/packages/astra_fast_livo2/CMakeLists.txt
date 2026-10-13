cmake_minimum_required(VERSION 3.8)
project(astra_fast_livo2)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/astra_fast_livo2_adapter.py scripts/measure_stability.py
  scripts/measure_trajectory.py scripts/measure_endurance.py scripts/audit_trajectory.py
  scripts/local_mapping_guard.py scripts/audit_planner_trajectory.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
