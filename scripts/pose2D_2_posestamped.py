#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
import tf

class Pose2D_2_PoseStamped():
    def __init__(self):
        rospy.init_node("pose2D_2_posestamped_node", anonymous=True)
        self.pose2Dsub = rospy.Subscriber("/pose2D", Pose2D, self.convert_2_pose_stamped, queue_size=1)
        self.pose_stamped_pub = rospy.Publisher('/laser_scanner_pose', PoseStamped, queue_size=1)
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
        self.pose_stamped_pub.publish(pose_stamped_msg)

if __name__=="__main__":
    node = Pose2D_2_PoseStamped()
