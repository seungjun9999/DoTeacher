cmake_minimum_required(VERSION 3.8)
project(doteacher_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)  # sensor_msgs 패키지 추가

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SteerThrottle.msg"
  "srv/UploadPicture.srv"
  "srv/RPiCommand.srv"
  "srv/DetectedImage.srv"
  "srv/DetectPoseService.srv"
  "srv/Nav2Index.srv"
  DEPENDENCIES sensor_msgs  # sensor_msgs 의존성 추가
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
