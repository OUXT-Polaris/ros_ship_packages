#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
import rospkg

class plotter:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')
        self.fix_sub = rospy.Subscriber("/fix", NavSatFix, self.fix_callback)
    def fix_callback(self,data):
        data.latitude
        data.longitude

if __name__ == '__main__':
    rospy.init_node('kml_plotter_node')
    gps_plotter = plotter()
    rospy.spin()
