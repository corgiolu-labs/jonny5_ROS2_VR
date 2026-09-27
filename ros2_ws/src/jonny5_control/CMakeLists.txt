cmake_minimum_required(VERSION 3.16)
project(jonny5_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wno-sign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Protocol codec + transports: no ROS dependency, unit-tested on its own.
add_library(jonny5_protocol STATIC
  src/j5_protocol_v2.cpp
  src/spi_transport.cpp
)
target_include_directories(jonny5_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
set_target_properties(jonny5_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(jonny5_control SHARED
  src/jonny5_system.cpp
)
target_link_libraries(jonny5_control PUBLIC
  jonny5_protocol
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface jonny5_control.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS jonny5_protocol jonny5_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_j5_protocol_v2 test/test_j5_protocol_v2.cpp)
  target_link_libraries(test_j5_protocol_v2 jonny5_protocol)
  ament_add_gtest(test_jonny5_system test/test_jonny5_system.cpp)
  target_link_libraries(test_jonny5_system jonny5_control)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
