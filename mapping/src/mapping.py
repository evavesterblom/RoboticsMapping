import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import tf_conversions
import math
import numpy as np

from utils.transform import world_to_grid
from utils.math import parametric_equation

class RollingBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = [0.5]
        self.index = 0

    def add(self, value):
        if len(self.buffer) < self.size:
            self.buffer.append(value)
        else:
            self.buffer[self.index] = value
        self.index += 1
        if self.index >= self.size:
            self.index = 0

    def get(self):
        return self.buffer

    def get_average(self):
        return sum(self.buffer) / len(self.buffer)

class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.buffer = RollingBuffer(100)

    def measure(self, value):
        self.buffer.add(value)

    def get_value(self):
        return self.buffer.get_average()
    
def get_point_location(position, heading, angle, distance): #lidar (0.24, 0, 0.3)
    x = position[0] + distance * math.cos(heading + angle) + 0.24
    y = position[1] + distance * math.sin(heading + angle)
    return x, y

class Mapper:
    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.position = (0, 0)
        self.heading = 0
        self.laser_subscriber = rospy.Subscriber('/laser_scan', LaserScan, self.laser_callback, queue_size=1, buff_size=2**24)
        self.position_subscriber = rospy.Subscriber('/odom', Odometry, self.position_callback)
        self.map_publisher = rospy.Publisher('/map_buf', OccupancyGrid, queue_size=10)
        self.lidar_publisher = rospy.Publisher('/lidar_points_buf', MarkerArray, queue_size=10)
        self.lidar = MarkerArray()
        self.map = np.array([[Cell(x, y) for x in range(int(width / resolution))] for y in range(int(height / resolution))], Cell)
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_orientation = tf_conversions.transformations.quaternion_from_euler(0, math.pi, -math.pi/2)

    def laser_callback(self, data):
        self.process_map(data)
        self.lidar_publisher.publish(self.lidar)
        self.map_publisher.publish(self.get_map())

    def process_map(self, data):
        self.lidar = MarkerArray()
        local_position = self.position
        local_heading = self.heading
        found_points = set()
        for i in range(len(data.ranges)):
            if not(data.range_min < data.ranges[i] < data.range_max):
                continue
            x, y = get_point_location(local_position, 
                                      local_heading, 
                                      data.angle_min + i * data.angle_increment, 
                                      data.ranges[i])
            found_points.add((x, y))
        for x, y in found_points:
            self.lidar.markers.append(self.get_marker(x, y, (1, 0, 0, 1)))
            result = world_to_grid(x, y, self.origin_x, self.origin_y, self.width, self.height, self.resolution)
            if result is not None:
                gx, gy = result
                self.map[gx, gy].measure(1)
                for point in parametric_equation((local_position[0], local_position[1]), (x, y), 100):
                    map_point = world_to_grid(point[0], point[1], self.origin_x, self.origin_y, self.width, self.height, self.resolution)
                    if map_point is not None and map_point != (gx, gy) and map_point not in found_points:
                        self.map[map_point[0], map_point[1]].measure(0)

    def get_marker(self, x, y, color):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lidar"
        marker.id = len(self.lidar.markers)
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

    def get_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.info.width = self.map.shape[0]
        msg.info.height = self.map.shape[1]
        msg.info.resolution = self.resolution
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.x = self.origin_orientation[0]
        msg.info.origin.orientation.y = self.origin_orientation[1]
        msg.info.origin.orientation.z = self.origin_orientation[2]
        msg.info.origin.orientation.w = self.origin_orientation[3]
        msg.info.map_load_time = rospy.Time.now()
        for idx, value in np.ndenumerate(self.map):
            msg.data.append(int(value.get_value() * 100))
        return msg

    def position_callback(self, data):
        self.position = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.heading = tf_conversions.transformations.euler_from_quaternion((data.pose.pose.orientation.x,
                                                                             data.pose.pose.orientation.y,
                                                                             data.pose.pose.orientation.z,
                                                                             data.pose.pose.orientation.w))[2]
if __name__ == '__main__':
    rospy.init_node('mapping')
    mapper = Mapper(60, 60, 0.1, -30, -30)
    rospy.loginfo("Mapping node started")
    rospy.spin()
