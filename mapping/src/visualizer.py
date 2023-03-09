import rospy
from mapping_msgs.msg import map_msg

import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self):
        self.map_subscriber = rospy.Subscriber('/map', map_msg, self.callback)

    def callback(self, data):
        global counter
        plt.plot([x.x for x in data.targets], [x.y for x in data.targets], 'ro')
        plt.plot(data.robot_pos.x, data.robot_pos.y, 'bo')
        plt.draw()
        counter += 1
        # plt.pause(0.00001)

def callback(data):
    global counter
    plt.plot([x.x for x in data.targets], [x.y for x in data.targets], 'ro')
    plt.plot(data.robot_pos.x, data.robot_pos.y, 'bo')
    plt.draw()
    counter += 1
    plt.pause(0.00001)

if __name__ == '__main__':
    counter = 0
    rospy.init_node('visualizer')
    # visualizer = Visualizer()
    map_subscriber = rospy.Subscriber('/map', map_msg, callback)
    plt.ion()
    plt.show()
    rospy.loginfo("Visualizer node started")
    rospy.spin()
