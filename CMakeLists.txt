cmake_minimum_required(VERSION 3.5)
project(ti_mmwave_rospkg)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarScan.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

add_library(mmwave
  src/mmWaveDataHdl.cpp
  src/mmWaveCommSrv.cpp
  src/DataHandlerClass.cpp
  src/ParameterParser.cpp
)
target_include_directories(mmwave PUBLIC
  ${PCL_INCLUDE_DIRS}
  include
)
target_link_libraries(mmwave
  ${PCL_LIBRARIES}
  Boost::system
  Threads::Threads
)
ament_target_dependencies(mmwave
  rclcpp
  serial
  std_msgs
  sensor_msgs
  message_filters
  pluginlib
  cv_bridge
)

add_executable(ti_mmwave_rospkg src/mmWaveDataHdl.cpp)
target_link_libraries(ti_mmwave_rospkg mmwave)
add_executable(mmWaveQuickConfig src/mmWaveQuickConfig.cpp)
target_link_libraries(mmWaveQuickConfig mmwave)

install(TARGETS
  ti_mmwave_rospkg
  mmWaveQuickConfig
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS mmwave
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch cfg rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()