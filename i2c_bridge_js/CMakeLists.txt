cmake_minimum_required(VERSION 3.8)
project(i2c_bridge_js)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/i2c_bridge.js
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch node_modules
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
