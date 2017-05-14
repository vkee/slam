#!/usr/bin/env python
# Code for calculating delta(state) after moving.
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



class deltaOdomCalc():
	def __init__(self):
		# initialize the node:
		rospy.init_node("delta_odom_calc", anonymous=True)
		# import parameters
		self.pastPose = None
		self.currPose = None
		self.estimatedPose = None
		# create trigger variables
		self.distanceTraveled = 0
		self.yawAccumulated = 0
		# poses initialized flag:
		self.posesInitialized = False
		# comparison values:
		self.distanceThreshold = rospy.get_param('robot_odom_distance_threshold') # meters
		angleThreshold = rospy.get_param('robot_odom_yaw_threshold')
		self.yawThreshold = (angleThreshold)*(math.pi/180) # radians
		# initialize subscribers
		# Suscribe to the laser scan topic
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		# initialize publishers
		# ---- For publishing the delta state ---------
		delta_state = rospy.get_param('odom_meas_topic', False)
		self.deltas_publisher = rospy.Publisher(delta_state, Pose2D)
		# ---- For sanity check with rviz ---------
		self.poseEstimateDeltas_publisher = rospy.Publisher('delta_estimated_state', PoseStamped)
		# testing array
		self.deltaHistory = []
		self.initialPose = [0,0,0]
		# loop
		rospy.spin()


	def convert_2_pose_stamped(self, data):
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


	def odom_callback(self, msg):
		'''
		Input:
			msg - nav_msgs/Odometry
		Returns:
			???
		'''
		if not(self.posesInitialized):
			quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
			eulAng_orientation = tf.transformations.euler_from_quaternion(quaternion)
			self.pastPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
			self.currPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
			self.posesInitialized = True
			return None
		# accumulate distance and yaw:
		self.distanceTraveled += math.sqrt( (msg.pose.pose.position.x - self.currPose[0])**2 + (msg.pose.pose.position.y - self.currPose[1])**2 )
		quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
		eulAng_orientation = tf.transformations.euler_from_quaternion(quaternion)
		self.yawAccumulated +=  abs(eulAng_orientation[2] - self.currPose[2])
		# check flags:
		if self.distanceTraveled >= self.distanceThreshold or self.yawAccumulated >= self.yawThreshold:
			#publish the deltas:
			deltas = Pose2D()
			deltas.x = msg.pose.pose.position.x - self.pastPose[0]
			deltas.y = msg.pose.pose.position.y - self.pastPose[1]
			# store in array:
			self.deltaHistory.append(deltas)
			# Testing: ------
			# deltas.x = 
			# deltas.y = 
			# Testing ------
			deltas.theta = eulAng_orientation[2] - self.pastPose[2]
			deltas.x = deltas.x/math.cos(deltas.theta)
			deltas.y = deltas.y/math.sin(deltas.theta)
			self.deltas_publisher.publish(deltas)
			estimatedPose = Pose2D()
			for e in self.deltaHistory:
				estimatedPose.x += e.x
				estimatedPose.y += e.y
				estimatedPose.theta += e.theta
			# estimatedPose.x = self.pastPose[0] + deltas.x
			# estimatedPose.y = self.pastPose[1] + deltas.y
			# estimatedPose.theta = self.pastPose[2] + deltas.theta
			convertedEstimatedPose = self.convert_2_pose_stamped(estimatedPose)
			# print convertedEstimatedPose
			self.poseEstimateDeltas_publisher.publish(convertedEstimatedPose)
			# update the old pose
			self.pastPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
			# reset threshold:
			self.distanceTraveled = 0
			self.yawAccumulated = 0
		quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
		eulAng_orientation = tf.transformations.euler_from_quaternion(quaternion)
		self.currPose = [msg.pose.pose.position.x, msg.pose.pose.position.y, eulAng_orientation[2]]
		return None


if __name__=="__main__":
    node = deltaOdomCalc()







