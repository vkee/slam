#!/usr/bin/env python

# Code for converting the laser scan into point cloud




#!/usr/bin/env python
# GOD! THESE IS DEF A MUST!
from __future__ import division
# ROS' heart:
import rospy
# for creating the .png images for the map
import png
# for working our linear algebra magic
import numpy as np
# for handling the matrix/image conversions:
from skimage.io import imread, imsave
# for dealing with coordinate transforms:
import tf
# for copying stuff
import copy
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



def laserScan_callback(msg):
	'''
	Input: 
		msg - LaserScan
	Returns:
		PointCloud2
	'''
	global wallE, pointCloud2_publisher
	# publish the point cloud
	pointCloud2_publisher.publish(wallE.projectLaser(msg))








'''
==========================================
======              MAIN             =====
==========================================
'''
print ""
print " ==== About to start main ==== "
print ""
# Initialize our node with the name:
rospy.init_node('laserToPointCloud2Converter')
# create global variable with time 
now = rospy.get_rostime()
rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
currentTime = now.secs + now.nsecs*1e-9


# create our converter object:
wallE = LaserProjection()



'''
-------------------------------
---    Topic Subscribers!   ---
-------------------------------
'''
# Suscribe to the laser scan topic
laserScan_sub = rospy.Subscriber('scan', LaserScan, laserScan_callback)


'''
-------------------------------
---    Topic Publishers!    ---
-------------------------------
'''
# ---- For publishing the point cloud ---------
pointCloud2_publisher = rospy.Publisher('cloud_in', PointCloud2)



'''
-------------------------------
---       Infinite Loop     ---
-------------------------------
'''
rospy.spin()
