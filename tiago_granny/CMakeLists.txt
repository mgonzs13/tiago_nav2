
cmake_minimum_required(VERSION 3.5)
project(tiago_granny)


find_package(ament_cmake REQUIRED)
find_package(tiago_nav2 REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY launch maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
