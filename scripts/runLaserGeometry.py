#!/usr/bin/env python

# Code for converting the laser scan into point cloud




#!/usr/bin/env python
# GOD! THESE IS DEF A MUST!
from __future__ import division
# ROS' heart:
import rospy
# for working our linear algebra magic
import numpy as np
# for sending out np arrays:
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# ROS messages
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# from laser_geometry class:
from laser_geometry import *


class LaserGeometryProjector():
	def __init__(self):
		laserScanTopic = rospy.get_param('laserscan_topic', False)
		rospy.init_node(laserScanTopic, anonymous=True)
		# create our converter object:
		self.wallE = LaserProjection()
		# subscribers:
		self.laserScan_sub = rospy.Subscriber('scan', LaserScan, self.laserScan_callback)
		# publishers:
		pointCloudTopic = rospy.get_param('laserscan_to_pointcloud_topic', False)
		self.pointCloud2_publisher = rospy.Publisher(pointCloudTopic, PointCloud2, queue_size=1)
		rospy.spin()


	def laserScan_callback(self, msg):
		'''
		Input: 
			msg - LaserScan
		Returns:
			PointCloud2
		'''
		# publish the point cloud
		self.pointCloud2_publisher.publish(self.wallE.projectLaser(msg))

if __name__=="__main__":
    node = LaserGeometryProjector()



