cmake_minimum_required(VERSION 3.5)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



add_executable(${PROJECT_NAME}_mocap_node
  natnet/natnet_packet_definition.cpp
  natnet/natnet_messages.cpp
  socket.cpp
  version.cpp
  mocap_node.cpp
  mocap_config.cpp
  data_model.cpp
  rigid_body_publisher.cpp
)

# include_directories(include /usr/local/include/)
# include_directories(include /opt/ros/dashing/include/)

target_link_libraries(${PROJECT_NAME}_mocap_node)
ament_target_dependencies(${PROJECT_NAME}_mocap_node "rclcpp" "rclcpp_components" "std_msgs" "tf2_ros" "tf2" "geometry_msgs" "tf2_msgs" "tf2_tools")
target_include_directories(${PROJECT_NAME}_mocap_node PUBLIC
  ../include
)

set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
                      OUTPUT_NAME mocap_node PREFIX "")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)

# install(TARGETS 
#   ${PROJECT_NAME}_mocap_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
