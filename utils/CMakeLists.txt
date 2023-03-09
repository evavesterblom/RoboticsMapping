cmake_minimum_required(VERSION 2.8.3)
project(utils)

find_package(catkin REQUIRED COMPONENTS
        rospy
)

catkin_package(
        CATKIN_DEPENDS rospy
)
catkin_python_setup()
catkin_install_python(PROGRAMS
        src/utils/transform.py src/utils/math.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )