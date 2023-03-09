cmake_minimum_required(VERSION 3.0.2)
project(mapping_msgs)

find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        message_generation
        geometry_msgs
        )
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_message_files(
        FILES
        coordinate_msg.msg
        map_msg.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime geometry_msgs/Point geometry_msgs/Pose2D
)