cmake_minimum_required(VERSION 3.16)
project(manipulation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/reset_peg.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
