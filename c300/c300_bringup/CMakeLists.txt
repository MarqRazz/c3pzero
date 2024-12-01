cmake_minimum_required(VERSION 3.5)
project(c300_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
