cmake_minimum_required(VERSION 3.8)
project(imu_driver)


add_library(driver SHARED src/driver.cpp)

target_include_directories(driver PUBLIC include/imu_driver)
target_link_libraries(driver PUBLIC i2c) # eck, this is a system library with no package config or cmake package
add_executable(driverapp_standalone src/main.cpp)

target_link_libraries(driverapp_standalone PUBLIC driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs tf2)

install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
