#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
import math
import tf

class SendPoseDelta():
    def __init__(self):

        rospy.init_node("pose2D_2_posestamped_node", anonymous=True)

        self.pose_correction = rospy.get_param('~pose_correction', False)

        self.estimated_pose = Pose2D()  # Will allow us to compare how scrapping the values is helping us
        self.got_first_pose = False

        self.prev_vel = Twist()
        self.prev_pose = Pose2D()

        self.prev_vel_time  = rospy.get_time()
        self.prev_pose_time = rospy.get_time()

        self.pose2D_sub = rospy.Subscriber("/pose2D", Pose2D, self.get_pose_delta, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.update_velocity, queue_size=1)
        self.pose_delta_pub = rospy.Publisher('/pose2D_detla', Pose2D, queue_size=1)
        self.pose_stamped_pub = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=1)

        rospy.spin()

    def convert_and_publish(self):
        """
        Converts the estimated Pose2D to a PoseStamped and publishes it for rviz viewing
        """
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = 'map'
        pose_msg = Pose()
        pose_msg.position.x = self.estimated_pose.x
        pose_msg.position.y = self.estimated_pose.y
        pose_msg.position.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.estimated_pose.theta)
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        pose_stamped_msg.pose = pose_msg
        self.pose_stamped_pub.publish(pose_stamped_msg)

    def update_velocity(self, data):
        vel_at = rospy.get_time()
        # print "Received velocity at"
        # print vel_at
        # print "Time since last velocity"
        # print vel_at - self.prev_vel_time

        # print "Meaured Velocity"
        # print data

        self.prev_vel_time = vel_at

        self.prev_vel = data


    def get_pose_delta(self, data):
        """ 
        Get the pose estimate from the laser_scan_matcher and compare
        the difference from the preveiously received pose to what was sent 
        for command velocity.  If they are very different send the command 
        velocity multiplied by the timestep between poses instead of the estimated pose.

        The velocity is received as just a linear and angular velocity,
        so we are simplifiying the displacement with linearization.
        """
        pose_at = rospy.get_time()
        # print "Received pose at"
        # print pose_at
        # print "Time since last pose"
        # print pose_at - self.prev_pose_time

        if not self.got_first_pose:
            self.estimated_pose = data
            self.got_first_pose = True

        delta = Pose2D(x=data.x-self.prev_pose.x, y=data.y-self.prev_pose.y, theta=data.theta-self.prev_pose.theta)
        # if one theta is negative and the other positive, we have to handle wraparound conditions
        if (data.theta < 0) != (self.prev_pose.theta < 0) and delta.theta > 1:
            print "Wrap around error for poses"
            print self.prev_pose
            print data
            delta.theta = (abs(data.theta) - math.pi) + (abs(self.prev_pose.theta) - math.pi)
            if data.theta > 0:
                # we went from negative to positive which should be a negative angular velocity
                delta.theta = -1* delta.theta
            print "new Delta"
            print delta



        # the delta from our velocity will be half the angular and then all the linear added in the new direction
        # will just be used for loose comparisons against the measured delta
        if self.pose_correction:
            calc_delta = Pose2D()
            calc_delta.theta = 1.0/2.0 * self.prev_vel.angular.x * 0.1  # sample rate is 10 Hz
            calc_delta.x = 1.0/1.0 * math.cos(self.prev_pose.theta + calc_delta.theta) * self.prev_vel.linear.x * 0.1
            calc_delta.y = 1.0/1.0 * math.sin(self.prev_pose.theta + calc_delta.theta) * self.prev_vel.linear.x * 0.1

            self.prev_pose = data
            self.prev_pose_time = pose_at

            # if the pose delta is way off want we predict, we don't want to send it
            xdiff = abs(delta.x - calc_delta.x)
            ydiff = abs(delta.y - calc_delta.y)
            thdiff = abs(delta.theta - calc_delta.theta)
            if ((xdiff > 2 * abs(calc_delta.x) and xdiff > 0.01) or (ydiff > 2 * abs(calc_delta.y) and ydiff > 0.01) or 
            (thdiff > 2 * abs(calc_delta.theta) and thdiff > 0.01)):
                print "POSE DELTA OFF"
                print ""
                print "MEASURED"
                print delta
                print "Calculated"
                print calc_delta

                self.estimated_pose = Pose2D(x=self.estimated_pose.x+1.0/1.0*calc_delta.x, y=self.estimated_pose.y+1.0/1.0*calc_delta.y, 
                                                theta=self.estimated_pose.theta+1.0/1.0*calc_delta.theta)
                self.convert_and_publish()
                return

        self.estimated_pose = Pose2D(x=self.estimated_pose.x+delta.x, y=self.estimated_pose.y+delta.y, 
                                        theta=self.estimated_pose.theta+delta.theta)
        self.convert_and_publish()

        self.pose_delta_pub.publish(delta)



if __name__=="__main__":
    node = SendPoseDelta()