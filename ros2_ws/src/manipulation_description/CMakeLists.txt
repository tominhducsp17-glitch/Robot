cmake_minimum_required(VERSION 3.16)
project(manipulation_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
