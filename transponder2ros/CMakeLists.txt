cmake_minimum_required(VERSION 3.8)
project(transponder2ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(iac_msgs REQUIRED)

# Includes
include_directories(
    include
)

# Add executables
add_executable(transponder2ros_node src/transponder2ros_node.cpp
    src/functions_ros.cpp
    src/functions_udp.cpp
    src/transponder2ros.cpp
    )
    ament_target_dependencies(transponder2ros_node 
        rclcpp
        std_msgs
        sensor_msgs
        iac_msgs
    )
    install(TARGETS
      transponder2ros_node
        DESTINATION lib/${PROJECT_NAME}
    )

# Copy over config and launch files
install(DIRECTORY
  # config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
