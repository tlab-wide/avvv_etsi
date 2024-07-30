cmake_minimum_required(VERSION 3.8)
project(visually)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(cpm_ros_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(GeographicLib REQUIRED)

add_library(transforms src/transforms.cpp)
ament_target_dependencies(transforms geometry_msgs tf2 tf2_eigen rviz_visual_tools)
target_include_directories(transforms SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} PRIVATE include PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(transforms ${GeographicLib_LIBRARIES})

add_library(entities src/entities.cpp)
target_link_libraries(entities transforms)
ament_target_dependencies(entities geometry_msgs visualization_msgs interactive_markers shape_msgs tf2 rviz_visual_tools ament_index_cpp tf2_eigen)
target_include_directories(entities PRIVATE include)

add_subdirectory(lib/csv2 lib/csv2/build)

add_library(heatmap src/heatmap.cpp include/visually/heatmap.hpp)
ament_target_dependencies(heatmap rclcpp geometry_msgs ament_index_cpp cpm_ros_msgs rviz_visual_tools)
target_link_libraries(heatmap entities csv2)
target_include_directories(heatmap PRIVATE include)

add_library(rviz_tools src/rviz_tools.cpp)
target_link_libraries(rviz_tools entities heatmap)
ament_target_dependencies(rviz_tools rclcpp interactive_markers rviz_visual_tools tf2_eigen)
target_include_directories(rviz_tools SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_include_directories(rviz_tools PRIVATE include)

add_library(pointcloud_tools src/pointcloud_tools.cpp)
ament_target_dependencies(pointcloud_tools rclcpp sensor_msgs PCL ament_index_cpp)
target_link_libraries(pointcloud_tools ${PCL_LIBRARIES})
target_include_directories(pointcloud_tools SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(pointcloud_tools PRIVATE include)

add_library(net_status src/net_status.cpp)
target_include_directories(net_status PRIVATE include)
ament_target_dependencies(net_status cpm_ros_msgs rviz_visual_tools)

add_library(manager src/manager.cpp)
target_link_libraries(manager rviz_tools heatmap net_status transforms)
ament_target_dependencies(manager rclcpp tf2_msgs tf2 geometry_msgs autoware_auto_perception_msgs cpm_ros_msgs tf2_eigen)
target_include_directories(manager SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_include_directories(manager PRIVATE include)

add_library(rosbag src/rosbag.cpp)
ament_target_dependencies(rosbag rclcpp ament_index_cpp rosbag2_cpp)
target_include_directories(rosbag PRIVATE include)

add_executable(main src/main.cpp)
target_link_libraries(main rviz_tools pointcloud_tools rosbag manager net_status)
ament_target_dependencies(main rclcpp autoware_auto_perception_msgs cpm_ros_msgs tf2_eigen)
target_include_directories(main SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_include_directories(main PRIVATE include)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main PUBLIC c_std_99 cxx_std_17) #Require C99 and C++17

install(TARGETS
  entities
  rviz_tools
  pointcloud_tools
  rosbag
  manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
