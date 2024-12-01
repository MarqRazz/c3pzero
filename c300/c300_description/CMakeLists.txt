cmake_minimum_required(VERSION 3.5)
project(c300_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
