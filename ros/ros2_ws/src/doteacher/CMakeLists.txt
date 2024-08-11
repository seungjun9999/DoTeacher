cmake_minimum_required(VERSION 3.8)
project(doteacher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  doteacher
  SHARED
  hardware/ackermann_jetson.cpp
)

target_compile_features(doteacher PUBLIC cxx_std_17)
target_include_directories(doteacher PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/doteacher>
)
ament_target_dependencies(
  doteacher PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface doteacher.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/doteacher
)
install(
  DIRECTORY config launch
  DESTINATION share/doteacher
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/gazebo_control
  DESTINATION share/doteacher
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/doteacher
)
install(TARGETS doteacher
  EXPORT export_doteacher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY worlds
  DESTINATION share/doteacher
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(doteacher_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_doteacher_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_doteacher_launch test/test_carlikebot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_doteacher HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
