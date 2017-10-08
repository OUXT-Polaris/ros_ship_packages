#!/usr/bin/env python
from nav_msgs.msg import Odometry
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class world_frame_pub:
    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.transform_msg = TransformStamped()
        self.utm_odom_sub = rospy.Subscriber("/odom", Odometry, self.utm_odom_callback)
    def utm_odom_callback(self,data):
        self.is_recieved = True
        self.transform_msg.header.stamp = rospy.Time.now()
        self.transform_msg.header.frame_id = data.header.frame_id
        self.transform_msg.child_frame_id = data.child_frame_id
        self.transform_msg.transform.translation.x = data.pose.pose.position.x
        self.transform_msg.transform.translation.y = data.pose.pose.position.y
        self.transform_msg.transform.translation.z = data.pose.pose.position.z
        self.transform_msg.transform.rotation.x = data.pose.pose.orientation.x
        self.transform_msg.transform.rotation.y = data.pose.pose.orientation.y
        self.transform_msg.transform.rotation.z = data.pose.pose.orientation.z
        self.transform_msg.transform.rotation.w = data.pose.pose.orientation.w
        self.broadcaster.sendTransform(self.transform_msg)

if __name__ == '__main__':
    rospy.init_node('world_frame_publisher', anonymous=False)
    publisher = world_frame_pub()
    rospy.spin()
