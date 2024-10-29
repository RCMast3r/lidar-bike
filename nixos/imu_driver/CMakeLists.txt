cmake_minimum_required(VERSION 3.8)
project(imu_driver)

add_executable(driverapp_standalone src/main.cpp src/driver.cpp)
target_include_directories(driverapp_standalone PUBLIC include/imu_driver)

target_link_libraries(driverapp_standalone PUBLIC i2c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(imu_node src/imu_node.cpp src/driver.cpp)
target_include_directories(imu_node PUBLIC include/imu_driver)

ament_target_dependencies(imu_node PUBLIC rclcpp sensor_msgs tf2)

target_link_libraries(imu_node PUBLIC i2c)

# install(
#   TARGETS driver
#   EXPORT export_driver
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ament_export_targets(export_driver HAS_LIBRARY_TARGET)
# ament_export_dependencies(driver)

install(TARGETS imu_node driverapp_standalone
  DESTINATION lib/${PROJECT_NAME})

ament_package()
