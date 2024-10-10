cmake_minimum_required(VERSION 3.9)


project(berrydriver)


add_library(driver SHARED src/driver.cpp)

target_link_libraries(driver PUBLIC i2c) # eck, this is a system library with no package config or cmake package

target_include_directories(driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS driver DESTINATION lib)

include(GNUInstallDirs)
install(TARGETS driver
  EXPORT driverTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION include
)

# NAMESPACE sets the namespace that your target will live within.
# you can have multiple targets under the same namespace
install(EXPORT driverTargets
  FILE driverTargets.cmake
  NAMESPACE driver::  
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/driver
)

# this looks for your .cmake.in file that you created in step 1.
# if you set the directory differently you may need to handle this differently
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/driverConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/driverConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/driver
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/driverConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/driver
)

# actually install your header files
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.h*"
    )


add_executable(driverapp src/main.cpp)

target_link_libraries(driverapp PUBLIC driver)

install(TARGETS driverapp DESTINATION bin)