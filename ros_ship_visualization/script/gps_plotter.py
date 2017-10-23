#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
import rospkg
import requests
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class plotter:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')
        self.latitudes = []
        self.longitudes = []
        self.google_statc_map_api_key = rospy.get_param("~google_statc_map_api_key")
        self.maptype = rospy.get_param("~maptype")
        self.zoom = rospy.get_param("~zoom")
        self.size_x = rospy.get_param("~size/x")
        self.size_y = rospy.get_param("~size/y")
        self.wam_v_marker_color = rospy.get_param("~wam_v_marker_color")
        self.start_point_marker_color = rospy.get_param("~start_point_marker_color")
        self.messages_per_plot = rospy.get_param("~messages_per_plot",10)
        self.path_color = rospy.get_param("~path_color","0xff0000ff")
        self.map_image_file = self.package_path + "/data/map_image.png"
        self.gps_seq = 0
        self.image_pub = rospy.Publisher(rospy.get_name()+"map_image",Image,queue_size=1)
        self.google_statc_map_api_request_pub = rospy.Publisher(rospy.get_name()+"google_statc_map_api_request",String,queue_size=1)
        self.bridge = CvBridge()
        self.fix_sub = rospy.Subscriber("/fix", NavSatFix, self.fix_callback)
    def fix_callback(self,data):
        if self.gps_seq%self.messages_per_plot == 0:
            self.latitudes.append(data.latitude)
            self.longitudes.append(data.longitude)
            self.build_request()
        self.gps_seq = self.gps_seq + 1
    def build_request(self):
        url = "https://maps.googleapis.com/maps/api/staticmap?"
        center_request = "&center=" + str(self.longitudes[-1]) + "," + str(self.latitudes[-1])
        zoom_request = "&zoom=" + str(self.zoom)
        size_request = "&size=" + str(self.size_x) + "x" + str(self.size_y)
        maptype_request = "&maptype=" + self.maptype
        wam_v_marker_request = "&markers=color:" + self.wam_v_marker_color + "%7Clabel:C%7C" + str(self.longitudes[-1]) + "," + str(self.latitudes[-1])
        start_point_marker_request = "&markers=color:" + self.start_point_marker_color + "%7Clabel:S%7C" + str(self.longitudes[0]) + "," + str(self.latitudes[0])
        path_request = self.build_path_request()
        api_key_request = "&key=" + self.google_statc_map_api_key
        request_url = url+center_request+zoom_request+size_request+maptype_request+path_request+start_point_marker_request+wam_v_marker_request+api_key_request
        request_msg = String()
        request_msg.data = request_url
        self.google_statc_map_api_request_pub.publish(request_msg)
        f=open(self.map_image_file,'wb')
        f.write(requests.get(request_url).content)
        f.close()
        self.publish_map_image()
    def build_path_request(self):
        num_points = len(self.longitudes)
        path_request = "&path=color:" + self.path_color + "|weight:5"
        for i in range(num_points):
            path_request = path_request + "|" + str(self.longitudes[-1]) + "," + str(self.latitudes[-1])
        return path_request

    def publish_map_image(self):
        map_image = cv2.imread(self.map_image_file)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(map_image, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('gps_plotter_node')
    gps_plotter = plotter()
    rospy.spin()
