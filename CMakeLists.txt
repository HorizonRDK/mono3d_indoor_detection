cmake_minimum_required(VERSION 3.5)
project(mono3d_indoor_detection)
#include(../../../../robot_dev_config/aarch64_toolchainfile.cmake)
#set(CMAKE_PREFIX_PATH "../../../../install/")
#set(CMAKE_CXX_FLAGS " -g ")
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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  ${CMAKE_SYSROOT}/usr/include/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/centernet_3d_detection_node.cpp
  src/image_utils.cpp
  src/centernet_3d_output_parser.cpp
)

add_executable(image_publisher
        src/image_publisher.cpp
        src/image_utils.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  ai_msgs
  OpenCV
)

ament_target_dependencies(
  image_publisher
  rclcpp
  dnn_node
  OpenCV
  sensor_msgs
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS image_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
