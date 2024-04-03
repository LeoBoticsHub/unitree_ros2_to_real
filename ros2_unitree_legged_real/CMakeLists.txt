cmake_minimum_required(VERSION 3.5)
project(ros2_unitree_legged_real)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

set(ROS_DEPS rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros std_srvs ros2_unitree_legged_msgs)

find_package(ros2_unitree_legged_msgs REQUIRED)

include_directories(
    include
)

link_directories(/usr/local/lib/libunitree_legged_sdk.so)

set(EXTRA_LIBS -pthread unitree_legged_sdk)

set(CMAKE_CXX_FLAGS "-O3")

# Install Python modules
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(unitree_service_handler)

ament_package()

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(unitree_high_ros2_control src/unitree_high_ros2_control.cpp)
target_link_libraries(unitree_high_ros2_control ${EXTRA_LIBS})
ament_target_dependencies(unitree_high_ros2_control ${ROS_DEPS})

add_library(unitree_ros2_client SHARED src/unitree_ros2_client.cpp)
target_link_libraries(unitree_ros2_client ${EXTRA_LIBS})
ament_target_dependencies(unitree_ros2_client ${ROS_DEPS})


# install
install(TARGETS
    ros2_udp
    ros2_walk_example
    ros2_position_example
    unitree_high_ros2_control
    DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS unitree_ros2_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

# install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )





