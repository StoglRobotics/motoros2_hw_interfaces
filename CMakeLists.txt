cmake_minimum_required(VERSION 3.8)
project(motoros2_hw_interfaces)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
# define ROS Package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  hardware_interface
  motoros2_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclpy
  std_msgs
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# GPIO System hardware interface
add_library(
  gpio_hardware_interface
  SHARED
  src/gpio_system.cpp
)
target_include_directories(
  gpio_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/motoros2_hw_interfaces>
)
ament_target_dependencies(gpio_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# MotorOS2 hardware interface
add_library(
  motoros2_hardware_interface
  SHARED
  src/motoros2_hardware_interface.cpp
)
target_include_directories(
  motoros2_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/motoros2_hw_interfaces>
)
ament_target_dependencies(motoros2_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

# FTS hardware interface
add_library(
  fts_hardware_interface
  SHARED
  src/fts_hardware_interface.cpp
)
target_include_directories(
  fts_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/motoros2_hw_interfaces>
)
ament_target_dependencies(fts_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  hardware_interface motoros2_hw_interfaces.xml
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_fts_hardware_interface test/test_fts_hardware_interface.cpp)
  target_include_directories(test_fts_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_fts_hardware_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    ros2_control_test_assets
  )

  ament_add_gmock(test_gpio_system test/test_gpio_system.cpp)
  target_include_directories(test_gpio_system PRIVATE include)
  ament_target_dependencies(
    test_gpio_system
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    ros2_control_test_assets
  )

  ament_add_gmock(test_motoros2_hardware_interface test/test_motoros2_hardware_interface.cpp)
  target_include_directories(test_motoros2_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_motoros2_hardware_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/motoros2_hw_interfaces
)

install(
  TARGETS gpio_hardware_interface
  EXPORT export_gpio_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS motoros2_hardware_interface
  EXPORT export_motoros2_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS fts_hardware_interface
  EXPORT export_fts_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_gpio_hardware_interface HAS_LIBRARY_TARGET)
ament_export_targets(export_motoros2_hardware_interface HAS_LIBRARY_TARGET)
ament_export_targets(export_fts_hardware_interface HAS_LIBRARY_TARGET)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
