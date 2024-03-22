cmake_minimum_required(VERSION 3.8)
project(kwr75_force_sensor_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(serial REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial_driver REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial_driver
)

add_library(
  kwr75_force_sensor_plugin
  SHARED
  src/hardware_interface.cpp
)
target_include_directories(
  kwr75_force_sensor_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  kwr75_force_sensor_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS kwr75_force_sensor_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  kwr75_force_sensor_plugin
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(test_kwr75_force_sensor src/kwr75_driver.cpp)
target_include_directories(
  test_kwr75_force_sensor
  PRIVATE
  include
)
ament_target_dependencies(
  test_kwr75_force_sensor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
install(TARGETS
  test_kwr75_force_sensor
  DESTINATION lib/${PROJECT_NAME})
 
install(DIRECTORY config launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
