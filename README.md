# slam MASTER README
SLAM Implementation for 6.834 Grand Challenge 2017

Read this README before reading the README in the `slam_backend` repo. 

# Setup

This project has the following external dependencies:
	- AprilTags
	- Octomaps
	- GTSAM/iSAM2

Installing AprilTags
	- Clone from https://github.com/RIVeR-Lab/apriltags_ros into catkin_ws/src/
	- cd into catkin_ws
	- catkin_make

Installing Octomap
	- sudo apt-get install ros-kinetic-octomap-server
	- (may need to install ros-kinetic-octomap and ros-kinetic-octomap-mapping and ros-kinetic-octomap-msgs)

Installing GTSAM/iSAM2
	- see slam_backend README

NOTE: Before launching SLAM, reset odom transform to the current robot location (Eric can do this). Make sure the robot is not moving

Launching everything
	- roslaunch slam slam_master.launch

RViz Setup
	- Open the config from `slam/rviz/odomSLAM.rviz`

# Debugging
Make sure your environment variables are set correctly:

```shell
export ROS_MASTER_URI=http://192.168.2.100:11311
export ROS_IP=<local-ip, from ifconfig>
source ~/catkin_ws/devel/setup.bash
```

# Guide to running from a ROS Bag
run roscore
	-- ```roscore```

run apriltags
	-- Make sure to copy run_apriltags_detection.launch from the slam/launch folder into the apriltags_ros/apriltags_ros/launch folder
	-- in slam folder ```roslaunch apriltags_ros run_apriltag_detection.launch```

run octomap
	-- Make sure to copy antonio_octomap.launch according to README in the slam repo
	-- in slam folder ```roslaunch octomap_server antonio_octomap.launch```

roslaunch slam
	-- ```roslaunch slam slam.launch```

run the bag
	-- ```rosparam set use_sim_time true```
	-- ```rosbag play --clock <bag_file>```


## Some other notes

Useful commands
```
export ROS_MASTER_URI=http://192.168.2.100:11311
export ROS_IP=<local-ip, from ifconfig>
source ~/catkin_ws/devel/setup.bash
env | grep ROS

rosrun 
rqt_graph
rviz
rosrun rf view_frames
```

# Outdated Notes:
## Occupancy grid:

We're using octomap_server to read from a cloud_in PointCloud2, which comes from the laserToPointCloud2Converter node (runLaserGeometry.py python file). To run the octomap server, you execute

```shell
roslaunch octomap_server antonio_octomap.launch
```

******* the file antonio_octomap.launch needs to be moved to the directory:
/opt/ros/indigo/share/octomap_server/launch (you can get here with `roscd octomap_server`)

### laserScan to pointCloud2:

Handled by the python script runLaserGeometry.py

### Resetting the map

To do that, import the Empty service and create an object with:
```python
from std_srvs.srv import Empty
resetter = rospy.ServiceProxy('/octomap_server/reset', Empty)
```
