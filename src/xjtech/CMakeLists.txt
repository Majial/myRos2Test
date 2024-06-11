cmake_minimum_required(VERSION 3.8)
project(xjtech)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_interfaces REQUIRED)
# 查找OpenCV
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)



# 头文件
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  )

add_executable(
  MainTest
  src/xjtech_main.cpp
  src/XjtechRos2.cpp
)

# ros2库
ament_target_dependencies(
  MainTest
  rclcpp
  sensor_msgs
  message_interfaces
  cv_bridge
)

# 动态库
set(xjtechlib lib)
message(STATUS "Linking libraries...")
target_link_libraries(
  MainTest
  ${PROJECT_SOURCE_DIR}/${xjtechlib}/libXIRStream.so
  ${PROJECT_SOURCE_DIR}/${xjtechlib}/libxjtech.so
  ${OpenCV_LIBS}
)

# 安装动态库文件
message(STATUS "Installing libraries...")
install(
  DIRECTORY ${xjtechlib}/
  DESTINATION lib/
)

# 安装可执行文件
message(STATUS "Installing executable...")
install(TARGETS
  MainTest
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
message(STATUS "Installing launch...")
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

# 安装yaml文件
message(STATUS "Installing config...")
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )



ament_package()
