cmake_minimum_required(VERSION 3.8)
project(jonny5_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
