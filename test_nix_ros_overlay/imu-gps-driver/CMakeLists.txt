cmake_minimum_required(VERSION 3.9)


project(berrydriver)


add_library(driver SHARED src/driver.cpp)

target_include_directories(driver PUBLIC include)
target_link_libraries(driver PUBLIC i2c) # eck, this is a system library with no package config or cmake package
add_executable(driverapp src/main.cpp)

target_link_libraries(driverapp PUBLIC driver)