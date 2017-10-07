# ros_ship_packages  
USV simulator for ROS  

# robot model
## ros_ship
![simulation on gazebo](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/gazebo.png)  
![simulation result on rviz](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/rviz.png)  

### simulated sensors
camera,VLP-16

### how to launch
roslaunch ros_ship_description ros_ship.launch

## WAM-V
![simulation on gazebo](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/wam-v_gazebo.png)  
![simulation result on rviz](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/wam-v_rviz.png)  
![demo](https://www.youtube.com/watch?v=tQ_12pDbhCQ&feature=youtu.be)

### simulated sensors
gps,imu,camera,VLP-16

### how to launch
roslaunch ros_ship_description wam-v.launch

# packages
## ros_ship_control  
ROS package for control simulated USV  

## ros_ship_description
ROS package for ros_ship_description(urdf,xacro,mesh,world files,etc..)  

## ros_ship_gazebo_plugins  
gazebo plugins for simlated USV  
### simple_buoyancy_plugin  
gazebo plugin for calculating buoyancy in a very simple way.  
### world_frame_publisher_plugin  
gazebo plugin for broadcast gazebo_world frame.  
