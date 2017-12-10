#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,Twist,Vector3
import tf
import numpy as np

class twist_publisher:
    def __init__(self):
        self.odom_pose = PoseStamped()
        self.odom_pose_before = PoseStamped()
        self.odom_pose_before_setted = False
        self.twist_pub = rospy.Publisher("/wam_v/twist", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_callback, queue_size=1)
    def odom_callback(self,data):
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header
        pose_stamped.pose = data.pose.pose
        if self.odom_pose_before_setted == False:
            self.odom_pose = pose_stamped
            self.odom_pose_before_setted = True
        else:
            self.odom_pose_before =  self.odom_pose
            self.odom_pose = pose_stamped
            self.publish_twist()
    def quaternion_to_euler(self,quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])
    def publish_twist(self):
        twist = Twist()
        duration = (self.odom_pose.header.stamp - self.odom_pose_before.header.stamp).to_sec()
        angular_pose = self.quaternion_to_euler(self.odom_pose.pose.orientation)
        angular_pose_before = self.quaternion_to_euler(self.odom_pose_before.pose.orientation)
        angular_diff = Vector3()
        angular_diff.x = angular_pose.x - angular_pose_before.x
        twist.angular.x = (angular_pose.x - angular_pose_before.x)/duration
        twist.angular.y = (angular_pose.y - angular_pose_before.y)/duration
        twist.angular.z = (angular_pose.z - angular_pose_before.z)/duration

        rot_matrix_x = np.matrix(
            [[1,0,0],
            [0,np.cos(angular_pose_before.x),-np.sin(angular_pose_before.x)],
            [0,np.sin(angular_pose_before.x), np.cos(angular_pose_before.x)]])
        rot_matrix_y = np.matrix(
            [[np.cos(angular_pose_before.y),0,np.sin(angular_pose_before.y)],
            [0,1,0],
            [-np.sin(angular_pose_before.y),0,np.cos(angular_pose_before.y)]])
        rot_matrix_z = np.matrix(
            [[np.cos(angular_pose_before.z),-np.sin(angular_pose_before.z),0],
            [np.sin(angular_pose_before.z),np.cos(angular_pose_before.z),0],
            [0,0,1]])
        rot_matrix_xyz = rot_matrix_x*rot_matrix_y*rot_matrix_z
        position = np.matrix([[self.odom_pose.pose.position.x],[self.odom_pose.pose.position.y],[self.odom_pose.pose.position.z]])
        position_before = np.matrix([[self.odom_pose_before.pose.position.x],[self.odom_pose_before.pose.position.y],[self.odom_pose_before.pose.position.z]])
        transformed_position = rot_matrix_xyz*position
        transformed_position_before = rot_matrix_xyz*position_before
        twist.linear.x = (transformed_position[0,0] - transformed_position_before[0,0])/duration
        twist.linear.y = (transformed_position[1,0] - transformed_position_before[1,0])/duration
        twist.linear.z = (transformed_position[2,0] - transformed_position_before[2,0])/duration
        #self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('twist_publisher_node', anonymous=True)
    publisher = twist_publisher()
    rospy.spin()
