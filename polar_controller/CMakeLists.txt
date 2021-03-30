cmake_minimum_required(VERSION 2.8.3)
project(polar_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        nav_msgs
        geometry_msgs
        )
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES joy2twist
        #  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(controller
        src/controller.cpp
        src/polar_controller.cpp
        )

target_link_libraries(controller
        ${catkin_LIBRARIES}
        )
