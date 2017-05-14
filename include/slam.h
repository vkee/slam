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

    // Publisher of estimated robot pose
    ros::Publisher est_robot_pose_pub_;

    // Converts a ROS Geometry Pose msg into a Pose2D struct Matrix3 (flattens the 6DoF pose to Pose2D)
    slam::Localization::Pose2D pose_stamped_msg_2_pose2d(geometry_msgs::PoseStamped pose_stamped_msg);

    // Converts a Pose2D struct into a ROS Geometry Pose msg
    geometry_msgs::PoseStamped pose2d_2_pose_stamped_msg(slam::Localization::Pose2D pose2d);

    // Converts a ROS Geometry Pose msg into an Eigen transform
    Eigen::Matrix4f pose_msg_2_transform(geometry_msgs::Pose pose_msg);

    // Converts a ROS Geometry Pose msg into a Pose2D struct (flattens the 6DoF pose to Pose2D)
    slam::Localization::Pose2D transform_2_pose2d(Eigen::Matrix4f transform);

    // Converts degrees to radians
    double deg_2_rad(double input);

    // Converts radians to degrees
    double rad_2_deg(double input);

    // GTSAM Localization library
    slam::Localization localization_;

    // The frame to display everything in
    std::string frame_id_;

    // Transform from robot base link to the camera frame
    Eigen::Matrix4f robot_base_T_cam_;

    // TF Object for dealing with reference frames
    tf::TransformListener tf_listener_;
    // The source frame for the tf transformLookup
    std::string source_frame_;
    // The target frame for the tf transformLookup
    std::string target_frame_;

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