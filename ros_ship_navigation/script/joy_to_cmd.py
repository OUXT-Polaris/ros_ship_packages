#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class joy_controlelr:
    def __init__(self):
        self.left_motor_cmd_pub = rospy.Publisher("/wam_v/left_motor_controller/command",Float64,queue_size=1)
        self.right_motor_cmd_pub = rospy.Publisher("/wam_v/right_motor_controller/command",Float64,queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback,queue_size=1)
    def joy_callback(self,data):
        self.left_motor_cmd = Float64()
        self.right_motor_cmd = Float64()
        self.left_motor_cmd.data = data.axes[1]
        self.right_motor_cmd.data = data.axes[4]
        self.left_motor_cmd_pub.publish(self.left_motor_cmd)
        self.right_motor_cmd_pub.publish(self.right_motor_cmd)

if __name__ == '__main__':
    rospy.init_node('joy_controlelr_node', anonymous=True)
    controller = joy_controlelr()
    rospy.spin()
