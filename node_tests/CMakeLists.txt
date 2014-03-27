cmake_minimum_required(VERSION 2.8.3)
project(node_tests)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
            roscpp
            rospy
            std_msgs
            tf
            rostest
            rosbag
            node_tests_msgs
          )

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(bag_player_for_tests 
  src/bag_player_for_tests.cpp 
  )
target_link_libraries(bag_player_for_tests ${catkin_LIBRARIES})
add_dependencies(bag_player_for_tests node_tests_msgs_gencpp)
