cmake_minimum_required(VERSION 3.0.2)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
#        message_generation
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_message_files(
#        FILES
#        coordinate_msg.msg
#        map_msg.msg
#)
#
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#)

catkin_package(
        CATKIN_DEPENDS
        nav_msgs/OccupancyGrid
        rospy
        sensor_msgs/LaserScan
        nav_msgs/Odometry
        mapping_msgs/map_msg
        visualization_msgs/Marker
        visualization_msgs/MarkerArray
        utils
)

catkin_install_python(PROGRAMS
  src/mapping.py scripts/teleop_key.py src/visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)