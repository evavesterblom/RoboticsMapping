cmake_minimum_required(VERSION 3.0.2)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS nav_msgs/OccupancyGrid rospy sensor_msgs/LaserScan nav_msgs/Odometry
)
catkin_install_python(PROGRAMS
  src/mapping.py scripts/teleop_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)