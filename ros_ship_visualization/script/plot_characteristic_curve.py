#!/usr/bin/env python
import rospy
from ros_ship_visualization.srv import *
from matplotlib import pyplot
import rospkg
import numpy as np

class plot_characteristic_curve_server:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')
        self.server = rospy.Service('/plot_characteristic_curve', PlotCharacteristicCurve, self.plot)
    def plot(self,req):
        file_path = self.package_path + "/data/" + req.file_name
        self.set_param(req.fluid_density,req.turning_radius,req.k0,req.k1,req.k2)
        inflow_rates = np.arange(req.min_inflow_rate,req.max_inflow_rate,req.resolution_inflow_rate)
        rotational_speeds = np.arange(req.min_rotational_speed,req.max_rotational_speed,req.resolution_rotational_speed)
        for inflow_rate in inflow_rates:
            thrusts = []
            for rotational_speed in rotational_speeds:
                thrusts.append(self.get_thrust(rotational_speed,inflow_rate))
            pyplot.plot(rotational_speeds, thrusts,label=("ship speed ="+str(inflow_rate)+"[m/s]"))
        pyplot.legend(loc='upper left')
        pyplot.xlabel("rotational speed [rad/s]")
        pyplot.ylabel("thrust [N]")
        pyplot.grid()
        pyplot.savefig(file_path+".jpg")
        pyplot.savefig(file_path+".eps")
        pyplot.close()
        rospy.loginfo("save characteristic curve server to "+file_path+" .jpg and .eps")

        return PlotCharacteristicCurveResponse()
    def set_param(self,fluid_density,turning_radius,k0,k1,k2):
        self.fluid_density = fluid_density
        self.turning_radius = turning_radius
        self.k0 = k0
        self.k1 = k1
        self.k2 = k2
    def get_thrust(self,rotational_speed,inflow_rate):
        if rotational_speed == 0:
            return 0
        if rotational_speed > 0:
            Js = inflow_rate/rotational_speed*self.turning_radius
            Kt = self.k2*Js*Js + self.k1*Js + self.k0
            thrust = self.fluid_density*pow(rotational_speed,2)*pow(self.turning_radius,4)*Kt
            return thrust
        if rotational_speed < 0:
            Js = inflow_rate/rotational_speed*self.turning_radius
            Kt = self.k2*Js*Js + self.k1*Js + self.k0
            thrust = -self.fluid_density*pow(rotational_speed,2)*pow(self.turning_radius,4)*Kt
            return thrust

if __name__ == '__main__':
    rospy.init_node('plot_characteristic_curve_server')
    plotter = plot_characteristic_curve_server()
    rospy.spin()
