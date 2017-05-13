#ifndef SLAM_H
#define SLAM_H

#include <ros/ros.h> // For ROS
#include <localization.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
// April Tags Msgs
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
// For tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "tf_conversions/tf_eigen.h"

class Slam
{
	public:
    Slam();
		~Slam();

		// The upper bound on the ROS loop rate
		int ros_loop_rate_;

	private:
		ros::NodeHandle nh_;

    void get_params();
    void setup_subs_pubs_srvs();

    // Initializes the GTSAM localization
    void init_localization();

    // Looks up transform from camera to the robot base link 
    // Returns true if successful
    bool lookup_camera_transform();

    // Subscriber for odometry measurements (preprocessed laser scan matcher relative poses)
    ros::Subscriber odom_meas_sub_;
    // Callback for receiving the odometry msg
    void odom_meas_cb(const geometry_msgs::Pose2DConstPtr& msg);

    // Subscriber for landmark measurements (from AprilTag ROS node)
    ros::Subscriber landmark_meas_sub_;
    // Callback for receiving the landmark measurement msg
    void land_meas_cb(const apriltags_ros::AprilTagDetectionArrayConstPtr& msg);

    // Converts a ROS Geometry Pose msg into a Pose2D struct Matrix3 (flattens the 6DoF pose to Pose2D)
    slam::Localization::Pose2D pose_stamped_msg_2_pose2d(geometry_msgs::PoseStamped pose_stamped_msg);

    // GTSAM Localization library
    slam::Localization localization_;

    // The frame to display everything in
    std::string frame_id_;

    // ID for the robot marker
    int robot_marker_id_;
    // ID for the landmark marker
    int landmark_marker_id_;

    // Counters for the robot pose and landmarks
    int robot_pose_counter_;
    int landmark_obs_counter_;

    double isam2_relinearize_thresh_;
    int isam2_relinearize_skip_;

    // Standard deviation of the translation and rotation components of the robot pose prior
    double prior_trans_stddev_;
    double prior_rot_stddev_;
    // Standard deviation of the translation and rotation components of the odometry measurements
    double odom_trans_stddev_;
    double odom_rot_stddev_;
    // Standard deviation of the translation and rotation components of the landmark observations
    double land_obs_trans_stddev_;
    double land_obs_rot_stddev_;
};

#endif // SLAM_H