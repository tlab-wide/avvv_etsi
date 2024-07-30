cmake_minimum_required(VERSION 3.8)
project(cpm_ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ItsPduHeader.msg"
  "msg/NetworkStatus.msg"
  "msg/OriginatingRSUContainer.msg"
  "msg/OriginatingVehicleContainer.msg"
  "msg/PerceivedObject.msg"
  "msg/PerceivedObjectContainer.msg"
  "msg/ReferencePosition.msg"
  "msg/StationDataContainer.msg"
  "msg/CPMManagementContainer.msg"
  "msg/CPMParameters.msg"
  "msg/CollectivePerceptionMessage.msg"
  "msg/CPMMessage.msg"
  "msg/CPMN.msg"
  DEPENDENCIES
    "builtin_interfaces"
    "tf2_msgs"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
