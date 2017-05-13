# slam
SLAM Implementation for 6.834 Grand Challenge 2017

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

### TODOs:
- Need to change "frame_id" from odom (current) to the origin of our SLAM implementation (e.g., slam_map).







## Implemenatation Info:
- Using apriltags_ros for range and bearing measurements. Need to get apriltags_ros package, and launch node with launch/run_apriltag_detection.launch launch file:

```shell
roslaunch apriltags_ros run_apriltag_detection.launch
```
