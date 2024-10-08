cmake_minimum_required(VERSION 3.8)
project(ros2_webots_sandbox)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bitbots_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(bitbots_docs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

set(INCLUDE_DIRS ${bitbots_msgs_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
  ${bitbots_docs_INCLUDE_DIRS} ${urdf_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${gazebo_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_msgs_LIBRARIES} ${rosgraph_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${bitbots_docs_LIBRARIES}
  ${urdf_LIBRARIES} ${nav_msgs_LIBRARIES} ${rclpy_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${gazebo_msgs_LIBRARIES})

ament_export_dependencies(bitbots_msgs)
ament_export_dependencies(rosgraph_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(bitbots_docs)
ament_export_dependencies(urdf)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(gazebo_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

install(DIRECTORY protos  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/start_simulator.py
  scripts/start_single.py
  scripts/start_webots_ros_supervisor.py
  DESTINATION lib/${PROJECT_NAME} )

ament_python_install_package(${PROJECT_NAME})

ament_package()
