#!/usr/bin/env python3

"""
Template for IAS0060 home assignment 2 Project 1 (SCITOS) .
Node to take a set of waypoints and to drive a differential
drive robot through those waypoints using a simple PID controller
and provided odometry data.

Students should complete the code. Note the places marked with "# TODO".

@author: Christian Meurer
@date: January 2022
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message;
         list of waypoints as MarkerArray message
"""

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker
import tf_conversions



################### TODO - teen edasi, aga juba tootab
############# enne kui liikuma hakkab keerab ennast oigesse suunda (yle vaiksema nurga)
############# et ei jookseks kokku kui wp on otsas
############# testida Kp, Ki ja Kd
############# kontrollida kas pid sai oigesti implementeeritud

class PIDController:
    """
    class for PID controller to calculated desired controller output
    """
    def __init__(self, dt):
        """
        class initialization
        @param: self
        @param: dt - time differential between control loops, determined
                     by updating frequency of the MotionController node
        @result: get controller gains from parameter server and
                 initialize class variables
        """
        ### timing ###
        self.dt = dt

        ### get controller gains as (2x1) vectors ###
        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")
        self.Ki = rospy.get_param("/controller_waypoints/controller/Ki")
        rospy.loginfo("-----------------------------------------------------")
        rospy.loginfo("Controller parameters:")
        rospy.loginfo("  Proportional gains: %.2f, %.2f"    ,self.Kp[0],self.Kp[1])
        rospy.loginfo("  Integral gains: %.2f, %.2f"        ,self.Ki[0],self.Ki[1])
        rospy.loginfo("  Derivative gains: %.2f, %.2f"      ,self.Kd[0],self.Kd[1])
        rospy.loginfo("-----------------------------------------------------")


        ### auxilary variables ###
        self.last_error = [0,0]
        self.int_error = [0,0]

    def control(self, error, dt):
        """
        control update of the controller class
        @param: self
        @param: e1 - (2x1) error vector for linear and angular position
        @result: cmd - (2x1) vector of controller commands
        """
        current_error = error
        int_error_lin = (current_error[0] - self.last_error[0]) * dt
        int_error_ang = (current_error[1] - self.last_error[1]) * dt
        self.int_error = [self.int_error[0] + int_error_lin, self.int_error[0] + int_error_ang]
        error_lin_d = 0
        error_ang_d = 0
        if dt != 0:
            error_lin_d = error[0]/dt
            error_ang_d = error[1]/dt


        #command = Kp e_p + Ki e_i + Kd e_d
        #e_p distance, angle error
        #e_i running error distance, angle in dt
        #e_d error change in dt

        e_p =   [self.Kp[0] * error[0],               self.Kp[1] *  error[1]]
        e_i =   [self.Ki[0] * self.int_error[0],      self.Ki[1] *  self.int_error[1]]
        e_d =   [self.Kd[0] * error_lin_d,            self.Kd[1] *  error_ang_d]

        self.last_error = error
        return  [e_p[0] + e_i[0] + e_d[0], e_p[1] + e_i[1] + e_d[1]]

    def zero(self):
        print("ZERO")
        self.int_error = [0,0]
        self.last_error = [0,0]

class MotionController:
    """
    main class of the node that performs controller updates
    """
    def __init__(self, rate):
        """
        class initialization
        @param: self
        @param: rate - updating frequency for this node in [Hz]
        @result: get static parameters from parameter server
                 (which have been loaded onto the server from the yaml
                 file using the launch file), to initialize the
                 controller, and to set up publishers and subscribers
        """
        self.dt = 1.0 / rate
        self.rate = rospy.Rate(rate)
        self.prev_time = rospy.Time.now().to_sec()
        
        self.odom_sub = rospy.Subscriber("/controller_diffdrive/odom", Odometry, self.onOdom)
        self.cmd_vel_pub = rospy.Publisher("/controller_diffdrive/cmd_vel", Twist, queue_size=10)
        self.waypoints_pub = rospy.Publisher("/mission_control/waypoints", MarkerArray, queue_size=10)

        self.odom_msg = None
        self.position = None # actual robot position
        self.heading = None # actual robot heading
        self.marker_array_msg = MarkerArray()
        self.twist_msg = Twist()
        self.vel_cmd = None # [linear, angular]

        self.delta_distance = 0.0
        self.delta_angle = 0.0
        self.error = [0,0]

        self.distance_margin = rospy.get_param("/controller_waypoints/mission/distance_margin")
        self.waypoints = rospy.get_param("/controller_waypoints/mission/waypoints")
        self.wp_count = len(self.waypoints)
        rospy.loginfo(" Mission parameters:")
        rospy.loginfo("  Distance margin: %.2f", self.distance_margin)
        rospy.loginfo("  Waypoints (#: x|y):")
        wp = 1
        for waypoint in self.waypoints:
            rospy.loginfo("   %d: %.1f|%.1f", wp, waypoint[0], waypoint[1])
            wp += 1
        rospy.loginfo("-----------------------------------------------------")

        self.wpIndex = 0 # counter for visited waypoints
        self.done_tracking = False

        self.pid = PIDController(self.dt)
        self.startTime = 0
        while self.startTime == 0:
            self.startTime = rospy.Time.now().to_sec()

    def run(self):
        """
        Main loop of the class
        @param: self
        @result: runs the step function for motion control update
        """
        while not rospy.is_shutdown():
            ### run only when odometry data is available and we still
            # have waypoints to reach ###
            if self.odom_msg and not self.done_tracking:
                self.step()
            # regulate motion control update according to desired timing
            self.rate.sleep()

    def step(self):
        """
        Perform an iteration of the motion control update where the
        desired velocity commands are calculated based on the defined
        controller and the calculated errors
        @param: self
        @result: publishes velocity commands as Twist message
        """
        ### check if current waypoint is reached and set new one if
        # necessary, additionally keep track of time required for
        # tracking ###
        if self.isWaypointReached():
            if not self.setNextWaypoint():
                if not self.done_tracking:
                    rospy.loginfo(f"This was the last waypoint in the list.")
                    endTime = rospy.Time.now().to_sec()
                    rospy.loginfo(f"Started node  [s]: {self.startTime}")
                    rospy.loginfo(f"Finished node [s]: {endTime}")
                    totalTime = endTime - self.startTime
                    rospy.loginfo(f"Elapsed time  [s]: {totalTime}")
                    self.done_tracking = True

        if not self.done_tracking:
            ### calculate error ###
            curr_position = self.position
            curr_heading = self.heading
            goal = self.waypoints[0]
            delta_x = (goal[0] - curr_position.x)
            delta_y = (goal[1] - curr_position.y)
            self.delta_distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2)) #P error X - distance error
            self.delta_angle = math.atan2(delta_y, delta_x) - curr_heading #P error Y - angle error
            self.error = [self.delta_distance, self.delta_angle]

            ### call controller class to get controller commands ###

            self.vel_cmd = self.pid.control(self.error, rospy.Time.now().to_sec() - self.prev_time)
            self.prev_time = rospy.Time.now().to_sec()

            print(f"Command: {self.vel_cmd}")
            

            ### publish cmd_vel (and marker array) ###
            self.publish_vel_cmd()
            self.publish_waypoints()

    def setNextWaypoint(self):
        """
        Removes current waypoint from list and sets next one as current target.
        @param: self
        @result: returns True if the next waypoint exists and has been set,
                 otherwise False
        """
        if not self.waypoints:
            return False

        self.waypoints.pop(0)

        if not self.waypoints:
            return False
        self.wpIndex += 1

        rospy.loginfo(f"----------------------------------------------")
        rospy.loginfo(f"                Next waypoint                 ")
        rospy.loginfo(f"----------------------------------------------")

        self.pid.zero()
        return True

    def isWaypointReached(self):
        """
        Checks if waypoint is reached based on pre-defined threshold.
        @param: self
        @result: returns True if waypoint is reached, otherwise False
        """
        if not self.waypoints:
            return False

        # TODO: calculate Euclidian (2D) distance to current waypoint
        if self.delta_distance < self.distance_margin:
            return True
        return False

    def publish_vel_cmd(self):
        """
        Publishes command velocities computed by the control algorithm.
        @param: self
        @result: publish message
        """
        # TODO: Your code here
        self.twist_msg.linear =Vector3(self.vel_cmd[0], 0, 0)  #Vector3(0, 0, 0)    # Vector3(self.vel_cmd[0], 0, 0) #here self.vel_cmd[0]
        self.twist_msg.angular = Vector3(0, 0, self.vel_cmd[1])#Vector3(0, 0, 0.4) # Vector3(0, 0, self.vel_cmd[1]) #here self.vel_cmd[1]
        self.cmd_vel_pub.publish(self.twist_msg)

    def onOdom(self, data):
        """
        Callback function that handles incoming Odometry messages and performs a partial quaternion to euler angle transformation to get the yaw angle theta
        @param: pose data stored in the odometry message
        @result: global variable pose_2D containing the planar
                 coordinates robot_x, robot_y and the yaw angle theta
        """
        self.odom_msg = data
        self.position = self.odom_msg.pose.pose.position
        q = self.odom_msg.pose.pose.orientation
        _, _, self.heading = tf_conversions.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])

    def publish_waypoints(self):
        """
        Helper function to publishe the list of waypoints, so that they can be visualized in RViz
        @param: self
        @result: publish message
        """
        self.marker_array = MarkerArray()
        marker_id = 0
        for waypoint in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.05
            marker.id = marker_id
            marker_id += 1
            self.marker_array.markers.append(marker)
        self.waypoints_pub.publish(self.marker_array)

if __name__ == '__main__':
    # initialize node and name it
    rospy.init_node("MotionController")
    try:
        controller = MotionController(10)
        controller.run()
    except rospy.ROSInterruptException:
        pass
