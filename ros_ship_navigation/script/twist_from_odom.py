#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class twist_calculator:
    def __init__(self):
        self.twist_pub = rospy.Publisher("/twist", Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback,queue_size=1)
        self.publish_enable = False
        self.last_elapsed_time = rospy.Time.now()
        self.now = rospy.Time.now()
        self.twist_msg = Twist()
    def odom_callback(self,data):
        self.twist_msg = data.twist.twist
        self.twist_pub.publish(self.twist_msg)

if __name__ == '__main__':
    rospy.init_node('twist_from_odom_node.py', anonymous=True)
    calculator = twist_calculator()
    rospy.spin()
