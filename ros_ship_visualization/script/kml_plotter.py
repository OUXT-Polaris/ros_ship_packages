#!/usr/bin/env python
import rospy
import simplekml
from sensor_msgs.msg import NavSatFix
import rospkg

class plotter:
    def __init__(self):
        self.kml = simplekml.Kml()
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')
        self.latitudes = []
        self.longitudes = []
        self.is_start_time = True
        self.last_recieved_data = NavSatFix()
        self.fix_sub = rospy.Subscriber("/fix", NavSatFix, self.fix_callback)
    def fix_callback(self,data):
        self.last_recieved_data = data
        #self.kml.newpoint(name=unicode(str(data.header.stamp), 'utf-8'), coords=[(data.latitude, data.longitude)])
        self.kml.newpoint(name=unicode("", 'utf-8'), coords=[(data.latitude, data.longitude)])
        if self.is_start_time == True:
            self.kml.newpoint(name=unicode("start point", 'utf-8'), coords=[(data.latitude, data.longitude)])
        self.is_start_time = False
        #self.latitudes.append(data.latitude)
        #self.longitudes.append(data.longitude)

    def save(self):
        self.kml.newpoint(name=unicode("finished point", 'utf-8'), coords=[(self.last_recieved_data.latitude, self.last_recieved_data.longitude)])
        #for i in range(len(self.longitudes)-1):
            #longitude = self.longitudes[i]
            #latitude = self.latitudes[i]
            #next_longitude = self.longitudes[i+1]
            #next_latitude = self.latitudes[i+1]
            #ls = self.kml.newlinestring(name='trajectory')
            #ls.coords = [(longitude,latitude,10.0), (next_longitude,next_latitude,10.0)]
            #ls.extrude = 1
            #ls.altitudemode = simplekml.AltitudeMode.relativetoground
        self.kml.save(self.package_path+"/data/gps_log.kml")
        rospy.loginfo("gps log was written in "+self.package_path+"/data/gps_log.kml")

if __name__ == '__main__':
    rospy.init_node('kml_plotter_node')
    kml_plotter = plotter()
    rospy.spin()
    kml_plotter.save()
