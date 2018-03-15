cmake_minimum_required(VERSION 2.8.3)
project(control_tool)

find_package(catkin REQUIRED
        roscpp
        cv_bridge
        pcl_conversions
        pcl_ros
        )
find_package(Qt5Widgets REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ui
        ${Qt5Widgets_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

add_library(control_mainwindow
        src/mainwindow.cpp
        include/mainwindow.h
        )

target_link_libraries(control_mainwindow ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## node
add_executable(control_node
        node/main.cpp
        )
target_link_libraries(control_node control_mainwindow ${PCL_LIBRARIES})

add_executable(save_node
        node/save_pcd_node.cpp)

target_link_libraries(save_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(save_index
#        node/save_txt_node.cpp)
#
#target_link_libraries(save_index ${PCL_LIBRARIES} ${catkin_LIBRARIES})
