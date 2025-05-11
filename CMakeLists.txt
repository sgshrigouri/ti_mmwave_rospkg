cmake_minimum_required(VERSION 3.8)
project(ti_mmwave_rospkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7.2 REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarScan.msg"
  "srv/mmWaveCLI.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# Declare library
add_library(mmwave
  src/mmWaveDataHdl.cpp
  src/mmWaveCommSrv.cpp
  src/DataHandlerClass.cpp
  src/mmWaveQuickConfig.cpp
  src/ParameterParser.cpp
)

target_link_libraries(mmwave
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${serial_LIBRARIES}
)

ament_target_dependencies(mmwave
  rclcpp std_msgs sensor_msgs pluginlib pcl_conversions serial
)

# Declare executables
add_executable(mmWaveLoader src/mmWaveLoader.cpp)
target_link_libraries(mmWaveLoader mmwave ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${serial_LIBRARIES})
ament_target_dependencies(mmWaveLoader rclcpp std_msgs sensor_msgs pluginlib pcl_conversions serial)

add_executable(mmWaveQuickConfig src/mmWaveQuickConfig.cpp)
target_link_libraries(mmWaveQuickConfig mmwave ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${serial_LIBRARIES})
ament_target_dependencies(mmWaveQuickConfig rclcpp std_msgs sensor_msgs pluginlib pcl_conversions serial)

# Link ROS 2 interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mmwave "${cpp_typesupport_target}")
target_link_libraries(mmWaveLoader "${cpp_typesupport_target}")
target_link_libraries(mmWaveQuickConfig "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  mmwave
  mmWaveLoader
  mmWaveQuickConfig
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch, config, and rviz files
install(DIRECTORY launch cfg rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(mmwave)
ament_export_dependencies(rclcpp std_msgs sensor_msgs pluginlib pcl_conversions serial rosidl_default_generators)

ament_package()