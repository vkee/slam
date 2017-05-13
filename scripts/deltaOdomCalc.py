#!/usr/bin/env python

# Code for calculating delta(state) after moving.




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
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Quaternion
import sensor_msgs.point_cloud2 as pc2
import tf
import math
# from laser_geometry class:
from laser_geometry import *

def convert_2_pose_stamped(data):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = 'map'
        pose_msg = Pose()
        pose_msg.position.x = data.x
        pose_msg.position.y = data.y
        pose_msg.position.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, data.theta)
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        pose_stamped_msg.pose = pose_msg
        return pose_stamped_msg


def odom_callback(msg):
	'''
	Input:
		msg - nav_msgs/Odometry
	Returns:
		???
	'''
	global posesInitialized, pastPose, currPose, distanceTraveled, yawAccumulated, deltas_publisher,poseEstimateDeltas_publisher
	if not(posesInitialized):
		quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
		eulAng_orientation = tf.transformations.euler_from_quaternion(quaternion)
		pastPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
		currPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
		posesInitialized = True
		return None
	# accumulate distance and yaw:
	distanceTraveled += math.sqrt( (msg.pose.pose.position.x - currPose[0])**2 + (msg.pose.pose.position.y - currPose[1])**2 )
	quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
	eulAng_orientation = tf.transformations.euler_from_quaternion(quaternion)
	yawAccumulated +=  abs(eulAng_orientation[2] - currPose[2])
	# check flags:
	if distanceTraveled >= distanceThreshold or yawAccumulated >= yawThreshold:
		#publish the deltas:
		deltas = Pose2D()
		deltas.x = msg.pose.pose.position.x - pastPose[0]
		deltas.y = msg.pose.pose.position.y - pastPose[1]
		deltas.theta = eulAng_orientation[2] - pastPose[2]
		deltas_publisher.publish(deltas)
		estimatedPose = Pose2D()
		estimatedPose.x = pastPose[0] + deltas.x
		estimatedPose.y = pastPose[1] + deltas.y
		estimatedPose.theta = pastPose[2] + deltas.theta
		convertedEstimatedPose = convert_2_pose_stamped(estimatedPose)
		# print convertedEstimatedPose
		poseEstimateDeltas_publisher.publish(convertedEstimatedPose)
		# update the old pose
		pastPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
		# reset threshold:
		distanceTraveled = 0
		yawAccumulated = 0
	quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
	eulAng_orientation = tf.transformations.euler_from_quaternion(quaternion)
	currPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
	return None








	

'''
==========================================
======              MAIN             =====
==========================================
'''
# Initialize our node with the name:
rospy.init_node('deltaOdomCalc')
# create variables:
pastPose = None
currPose = None
estimatedPose = None
# create global variable with time 
now = rospy.get_rostime()
rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
currentTime = now.secs + now.nsecs*1e-9
# create trigger variables
distanceTraveled = 0
yawAccumulated = 0
# poses initialized flag:
posesInitialized = False
# comparison values:
distanceThreshold = 0.2 # meters
yawThreshold = 5*(math.pi/180) # radians



'''
-------------------------------
---    Topic Subscribers!   ---
-------------------------------
'''
# Suscribe to the laser scan topic
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)


'''
-------------------------------
---    Topic Publishers!    ---
-------------------------------
'''
# ---- For publishing the delta state ---------
deltas_publisher = rospy.Publisher('delta_state', Pose2D)
# ---- For sanity check with rviz ---------
poseEstimateDeltas_publisher = rospy.Publisher('delta_estimated_state', PoseStamped)







'''
-------------------------------
---       Infinite Loop     ---
-------------------------------
'''
rospy.spin()
# while not rospy.is_shutdown():


# 	# deltaPose = pastPose - currPose


# 	rospy.sleep(1)






