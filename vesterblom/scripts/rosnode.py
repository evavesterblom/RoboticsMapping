import rospy
from geometry_msgs.msg import Twist

class Node:
    def __init__(self):
        self.subsriber = rospy.Subscriber("/turtle1/cmd_vel", Twist, self.callback)
        self.publisher = rospy.Publisher("/cmd_vel_received", Twist, queue_size=10)

    def callback(self, data):
        rospy.loginfo("Received and published: %s" % data)
        self.publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('rosnode')
    rospy.loginfo("Listener Publisher started")
    node = Node()
    rospy.spin()