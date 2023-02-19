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
import tf_conversions
from visualization_msgs.msg import MarkerArray, Marker


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
        self.dt = dt

        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")
        self.Ki = rospy.get_param("/controller_waypoints/controller/Ki")
        rospy.loginfo("-----------------------------------------------------")
        rospy.loginfo("Controller parameters:")
        rospy.loginfo("  Proportional gains: %.2f, %.2f",self.Kp[0],self.Kp[1])
        rospy.loginfo("  Integral gains: %.2f, %.2f",self.Ki[0],self.Ki[1])
        rospy.loginfo("  Derivative gains: %.2f, %.2f",self.Kd[0],self.Kd[1])
        rospy.loginfo("-----------------------------------------------------")

        ### auxilary variables ###
        self.last_error = np.zeros(2)
        self.int_error = np.zeros(2)

    def control(self, error_p, error_i, error_d):
        """
        control update of the controller class
        @param: self
        @param: e1 - (2x1) error vector for linear and angular position
        @result: cmd - (2x1) vector of controller commands
       
                  error * kP + totalError * kI + derivative * kD
                  https://www.vexforum.com/t/what-is-a-pid-controller/73678

        command = Kp e + Ki e_i + Kd e_dot
        e       = Vector2(distance err          , angle err)
        e_i     = Vector2(absement              , angle absement) I - running sum of previous errors
        e_dot   = Vector2(velocity change in dt , angle change in dt)

        """
        # Todo: Your code here
        # ...
        # cmd = ...

        # return cmd

        e =     [self.Kp[0] * error_p[0]    , self.Kp[1] * error_p[1]]
        e_i =   [self.Ki[0] * error_i[0]    , self.Ki[1] * error_i[1]]
        e_d =   [self.Kd[0] * error_d[0]    , self.Kd[1] * error_d[0]]
        return  [e[0] + e_i[0] + e_d[0]     , e[1] + e_i[1] + e_d[1]] 


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

        self.odom_sub = rospy.Subscriber("/controller_diffdrive/odom", Odometry, self.onOdom)
        self.cmd_vel_pub = rospy.Publisher("/controller_diffdrive/cmd_vel", Twist, self.twist_msg)
        self.waypoints_pub = rospy.Publisher("/mission_control/waypoints", MarkerArray, queue_size=10)

        self.odom_msg = None
        self.marker_array_msg = MarkerArray()
        self.twist_msg = Twist()
        self.twist_msg_term = 0
        self.vel_cmd = None #[linear, angular]

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

        self.wpIndex = 0    # counter for visited waypoints
        self.done_tracking = False

        self.pid = PIDController(self.dt)

        self.startTime = 0  # Registering start time of this node for performance tracking
        while self.startTime == 0:
            self.startTime = rospy.Time.now().to_sec()
        self.time = rospy.Time.now().to_sec()

        self.position = None
        self.heading = None

        #errors
        self.delta_distance = 0 #PX
        self.delta_angle = 0 #PY
        self.total_distance_absement = 0 #IX
        self.total_angle_absement = 0 #IY
        self.delta_lin_velocity = 0 #DX
        self.delta_ang_velocity = 0 #DY


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
            self.publish_waypoints()
            #TODO: Your code here

            ### calculate error ### done in odom

            ### call controller class to get controller commands ###
            self.vel_cmd = self.pid.control(error=[self.delta_distance, self.delta_angle], 
                                            error_i=[self.total_distance_absement, self.total_angle_absement], 
                                            error_d=[self.delta_lin_velocity, self.delta_ang_velocity])
            self.publish_vel_cmd()
            self.publish_waypoints()

            ### publish cmd_vel (and marker array) ###

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
        #if distance < self.distance_margin:
        #    return True
        return False

    def publish_vel_cmd(self):
        """
        Publishes command velocities computed by the control algorithm.
        @param: self
        @result: publish message
        """
        self.twist_msg.linear = Vector3(self.vel_cmd[0], 0, 0) #here self.vel_cmd[0]
        self.twist_msg.angular = Vector3(0, 0, self.vel_cmd[1]) #here self.vel_cmd[1]
        self.publish_vel_cmd.publish(self.twist_msg)
        #self.time = rospy.Time.now().to_sec() #kas on vaja?



    def onOdom(self, data):
        """
        Callback function that handles incoming Odometry messages and
        performs a partial quaternion to euler angle transformation to
        get the yaw angle theta
        @param: pose data stored in the odometry message
        @result: global variable pose_2D containing the planar
                 coordinates robot_x, robot_y and the yaw angle theta
        """
        prev_time = self.time
        current_time = rospy.Time.now().to_sec()
        delta_time = current_time - prev_time
        
        self.odom_msg = data

        prev_position = self.position
        prev_heading = self.heading
        
        self.position = self.odom_msg.pose.pose.position
        q = self.position.orientation
        _, _, self.heading = tf_conversions.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        
        curr_position = self.position
        curr_heading = self.heading

        #P error X - distance error
        goal = self.waypoints[0]
        delta_x = (goal[0] - curr_position.x)
        delta_y = (goal[1] - curr_position.y)
        self.delta_distance = math.sqrt( pow(delta_x, 2) + pow(delta_y, 2) )

        #P error Y - angle error
        self.delta_angle = math.atan2(delta_y, delta_x) -  curr_heading

        #I error X - absement distance - running sum of previous errors
        self.total_distance_absement += self.delta_distance * delta_time

        #I error Y - absement angle - running sum of previous errors
        self.total_angle_absement += self.delta_angle * delta_time

        #D error X - change linear velocity in dt
        delta_x_previous = (curr_position.x - prev_position.x)
        delta_y_previous = (curr_position.y - prev_position.y)
        delta_distance_previous = math.sqrt( pow(delta_x_previous, 2) + pow(delta_y_previous, 2) )
        if delta_time != 0:
            self.delta_lin_velocity = delta_distance_previous / delta_time

        #D error Y - change angular velocity in dt
        if delta_time != 0:
            self.delta_ang_velocity = (curr_heading - prev_heading) / delta_time




    def publish_waypoints(self):
        """
        Helper function to publishe the list of waypoints, so that they
        can be visualized in RViz
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
    rospy.init_node("MotionController")
    try:
        controller = MotionController(10)
        controller.run()
    except rospy.ROSInterruptException:
        pass
