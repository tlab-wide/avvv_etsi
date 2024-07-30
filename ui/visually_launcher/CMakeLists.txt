cmake_minimum_required(VERSION 3.5)

project(visually_launcher VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

message("Shit" ${QT_VERSION_MAJOR})

include_directories(
    "lib/common"
    "/opt/ros/humble/include/rcpputils"
    "/opt/ros/humble/include/rosbag2_storage"
    "/opt/ros/humble/include/rosidl_typesupport_introspection_cpp"
    "/opt/ros/humble/include/rosidl_runtime_cpp"
    "/opt/ros/humble/include/rosidl_typesupport_cpp"
    "/opt/ros/humble/include/rosidl_typesupport_interface"
    "/opt/ros/humble/include/rosidl_runtime_c"
    "/opt/ros/humble/include/rcutils"
    "/opt/ros/humble/include/rmw"
    "/opt/ros/humble/include/rcl"
    "/opt/ros/humble/include/rclcpp"
    "/opt/ros/humble/include/rosbag2_cpp"
    "/opt/ros/humble/include/ament_index_cpp"
)

link_directories(/opt/ros/humble/lib)

add_library(common
    lib/common/common.cpp
    lib/common/common.hpp
)
target_link_libraries(common
    Qt${QT_VERSION_MAJOR}::Widgets
    rosbag2_storage
    rosbag2_cpp
    ament_index_cpp
)

set(PROJECT_SOURCES
        src/main.cpp
        src/mainwindow.cpp
        src/mainwindow.hpp
        src/mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(visually_launcher
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET visually_launcher APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(visually_launcher SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(visually_launcher
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(visually_launcher PRIVATE Qt${QT_VERSION_MAJOR}::Widgets common)

set_target_properties(visually_launcher PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS visually_launcher
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(visually_launcher)
endif()
