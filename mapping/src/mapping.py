import rospy
from tf.transformations import euler_from_quaternion
import tf_conversions
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Mapper:
    def __init__(self):
        self.position = (0, 0)
        self.heading = 0
        self.laser_subscriber = rospy.Subscriber('/laser_scan', LaserScan, self.callback)
        self.position_subscriber = rospy.Subscriber('/odom', Odometry, self.position_callback)

    def callback(self, data):
        # rospy.loginfo(len(data.ranges))
        pass

    def position_callback(self, data):
        self.position = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.heading = tf_conversions.transformations.euler_from_quaternion((data.pose.pose.orientation.x,
                                                                             data.pose.pose.orientation.y,
                                                                             data.pose.pose.orientation.z,
                                                                             data.pose.pose.orientation.w))[2]
        rospy.loginfo(self.position)
        rospy.loginfo(self.heading)


if __name__ == '__main__':
    rospy.init_node('mapping')
    mapper = Mapper()
    rospy.loginfo("Mapping node started")
    rospy.spin()
