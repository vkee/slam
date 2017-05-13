#include "slam.h"

Slam::Slam(): robot_marker_id_(0), landmark_marker_id_(1)
{
  // Get params
  get_params();

  // Setup the subscribers and publishers
  setup_subs_pubs_srvs();

  // // Lookup camera transform
  // bool success = lookup_camera_transform();

  // if (success)
  // {
  //   // Initialize the GTSAM library
  //   init_localization();

  //   ROS_INFO("Slam Node Initialized");
  // }

  // else
  // {
  //   ROS_ERROR("Unable to find camera transform to base link, Slam node NOT initialized");
  // }

}

Slam::~Slam()
{
  ROS_FATAL("Slam Node Terminated");
}

void Slam::get_params()
{
  // the rate to run the node
  nh_.getParam("slam_ros_loop_rate", ros_loop_rate_);

  // frame for everything
  nh_.getParam("frame_id", frame_id_);

  // GTSAM_Localization Parameters
  // iSAM2 Parameters
  nh_.getParam("isam2_relinearize_thresh", isam2_relinearize_thresh_);
  nh_.getParam("isam2_relinearize_skip", isam2_relinearize_skip_);

  // Standard deviation of the translation components of the robot pose prior
  nh_.getParam("prior_trans_stddev", prior_trans_stddev_);
  // Standard deviation of the rotation components of the robot pose prior
  nh_.getParam("prior_rot_stddev", prior_rot_stddev_);

  // Standard deviation of the translation components of the odometry measurements
  nh_.getParam("odom_trans_stddev", odom_trans_stddev_);
  // Standard deviation of the rotation components of the odometry measurements
  nh_.getParam("odom_rot_stddev", odom_rot_stddev_);

  // Standard deviation of the translation components of the landmark observations
  nh_.getParam("land_obs_trans_stddev", land_obs_trans_stddev_);
  // Standard deviation of the rotation components of the landmark observations
  nh_.getParam("land_obs_rot_stddev", land_obs_rot_stddev_);
}

void Slam::setup_subs_pubs_srvs()
{
  // Subscriber for odometry measurements (preprocessed laser scan matcher relative poses)
  std::string odom_meas_topic;
  int odom_meas_queue_size;
  if (nh_.getParam("odom_meas_topic", odom_meas_topic) && nh_.getParam("odom_meas_queue_size", odom_meas_queue_size))
  {
    odom_meas_sub_ = nh_.subscribe(odom_meas_topic, odom_meas_queue_size, &Slam::odom_meas_cb, this);
    ROS_DEBUG("Odometry Measurement Topic %s Subscriber Loaded", odom_meas_topic.c_str());
  }
  else
  {
    ROS_FATAL("Odometry Measurement Topic subscriber not loaded");
  }

  // Subscriber for the landmark measurements (from AprilTag ROS node)
  std::string landmark_meas_topic;
  int landmark_meas_queue_size;
  if (nh_.getParam("landmark_meas_topic", landmark_meas_topic) && nh_.getParam("landmark_meas_queue_size", landmark_meas_queue_size))
  {
    landmark_meas_sub_ = nh_.subscribe(landmark_meas_topic, landmark_meas_queue_size, &Slam::land_meas_cb, this);
    ROS_DEBUG("Landmark Measurement Topic %s Subscriber Loaded", landmark_meas_topic.c_str());
  }
  else
  {
    ROS_FATAL("Landmark Measurement subscriber not loaded");
  }

  // Publisher for display pose estimate (PoseStamped so can see in RVIZ?)

}

// Looks up transform from camera to the robot base link 
// Returns true if successful
bool Slam::lookup_camera_transform()
{


  // tf::StampedTransform baseorig_T_camnow;
  // double curr_time = ros::Time::now().toSec();

  // // Get the tf transform
  // bool success = tf_listener_.waitForTransform(target_frame_, source_frame_, ros::Time(0), ros::Duration(5.0));

  // if (success)
  // {
  //   // Get the tf transform
  //   tf_listener_.lookupTransform(target_frame_, source_frame_, ros::Time(0), baseorig_T_camnow);
}

// Initializes the GTSAM localization
void Slam::init_localization()
{
  localization_.init_localization(isam2_relinearize_thresh_, isam2_relinearize_skip_, 
    prior_trans_stddev_, prior_rot_stddev_, odom_trans_stddev_, odom_rot_stddev_,
    land_obs_trans_stddev_, land_obs_rot_stddev_);
}

// Callback for receiving the odometry msg
void Slam::odom_meas_cb(const geometry_msgs::Pose2DConstPtr& msg)
{

}

// Callback for receiving the landmark measurement msg
void Slam::land_meas_cb(const apriltags_ros::AprilTagDetectionArrayConstPtr& msg)
{
  apriltags_ros::AprilTagDetectionArray detections_array_msg = *msg;
  std::vector<apriltags_ros::AprilTagDetection> detections = detections_array_msg.detections;

  // TODO: may need to get the most up to date robot pose so that landmark measurements are from a robot pose node where the robot is at at the time of the landmark measurement
  // apparently we are getting odometry measurements at 10 hz so that is probably fine?
  for (int i = 0; i < detections.size(); i++)
  {
    apriltags_ros::AprilTagDetection indiv_detection = detections[i];

    int landmark_id = indiv_detection.id;
    geometry_msgs::PoseStamped pose = indiv_detection.pose;

    // TODO: need to transform the measurement into the base link frame

    // Convert the PoseStamped to a Pose2D
    slam::Localization::Pose2D landmark_meas = pose_stamped_msg_2_pose2d(pose);

    // Updating the factor graph
    localization_.add_landmark_measurement(landmark_id, landmark_meas.x, landmark_meas.y, landmark_meas.theta);
  }

  // Optimize the factor graph
  localization_.optimize_factor_graph();




  // Publish pose estimate

}



// // Moves the robot, updates the state estimates, and updates the visualization
// bool Slam::move_robot(perception_msgs::OdomCommand::Request &req, perception_msgs::OdomCommand::Response &resp)
// {
//   ROS_INFO("Starting Move Robot Service Call...");

//   // Getting the robot's true pose
//   visualization_msgs::Marker::ConstPtr true_robot_marker_msg = ros::topic::waitForMessage<visualization_msgs::Marker>(true_robot_marker_topic_, nh_);
//   geometry_msgs::Pose true_robot_pose_msg = true_robot_marker_msg->pose;
//   // Transform from the global to the robot frame
//   Eigen::Matrix4f global_T_robot_curr = pose_msg_2_transform(true_robot_pose_msg);
//   std::cout << "global_T_robot_curr: " << global_T_robot_curr << std::endl;

//   // Get the commanded odometry from the service call
//   geometry_msgs::Pose commanded_odometry_msg = req.odom_command;
//   std::cout << "commanded_odometry: " << commanded_odometry_msg << std::endl;
//   // Transform from the current robot pose to the next robot pose
//   Eigen::Matrix4f robot_curr_T_robot_next = pose_msg_2_transform(commanded_odometry_msg);
//   // Transform from the global to the next robot pose
//   Eigen::Matrix4f global_T_robot_next = global_T_robot_curr * robot_curr_T_robot_next;
//   std::cout << "global_T_robot_next: " << global_T_robot_next << std::endl;

//   // Publish the true robot pose
//   true_robot_marker_pub_.publish(transform_2_robot_marker_msg(global_T_robot_next, true));

//   // Update factor graph and publish the estimated robot pose
//   Eigen::Matrix4f global_T_est_robot_pose = localization_.add_odom_measurement(robot_curr_T_robot_next);
//   est_robot_marker_pub_.publish(transform_2_robot_marker_msg(global_T_est_robot_pose, false));

//   ROS_INFO("Done handling Move Robot Service Call...");
// }

// // Takes a landmark measurement, updates the state estimates, and updates the visualization
// bool Slam::take_measurement(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
// {
//   ROS_INFO("Starting Take Measurement Service Call...");

//   // Getting the robot's true pose
//   visualization_msgs::Marker::ConstPtr true_robot_marker_msg = ros::topic::waitForMessage<visualization_msgs::Marker>(true_robot_marker_topic_, nh_);
//   geometry_msgs::Pose true_robot_pose_msg = true_robot_marker_msg->pose;
//   // Transform from the global to the robot frame
//   Eigen::Matrix4f global_T_robot = pose_msg_2_transform(true_robot_pose_msg);
//   std::cout << "global_T_robot_curr: " << global_T_robot << std::endl;

//   // Getting the landmarks's true pose
//   visualization_msgs::Marker::ConstPtr true_landmark_marker_msg = ros::topic::waitForMessage<visualization_msgs::Marker>(true_landmark_marker_topic_, nh_);
//   geometry_msgs::Pose true_landmark_pose_msg = true_landmark_marker_msg->pose;
//   // Transform from the global to the landmark frame
//   Eigen::Matrix4f global_T_landmark = pose_msg_2_transform(true_landmark_pose_msg);
//   std::cout << "global_T_landmark: " << global_T_landmark << std::endl;

//   // Transform from the robot to the landmark
//   Eigen::Matrix4f robot_T_landmark = global_T_robot.inverse() * global_T_landmark;

//   // Update factor graph and publish the estimated robot and landmark poses
//   Eigen::Matrix4f est_landmark_pose = localization_.add_landmark_measurement(robot_T_landmark);
//   est_landmark_marker_pub_.publish(transform_2_landmark_marker_msg(est_landmark_pose, false));
//   Eigen::Matrix4f est_robot_pose = localization_.get_est_robot_pose();
//   est_robot_marker_pub_.publish(transform_2_robot_marker_msg(est_robot_pose, false));

//   ROS_INFO("Done handling Take Measurement Service Call...");
// }

// Converts a ROS Geometry Pose msg into a Pose2D struct Matrix3 (flattens the 6DoF pose to Pose2D)
slam::Localization::Pose2D Slam::pose_stamped_msg_2_pose2d(geometry_msgs::PoseStamped pose_stamped_msg)
{
  // Convert landmark to transform
  geometry_msgs::Point position = pose_stamped_msg.pose.position;
  geometry_msgs::Quaternion quat = pose_stamped_msg.pose.orientation;

  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  slam::Localization::Pose2D pose2d;
  pose2d.x = position.x;
  pose2d.y = position.y;
  pose2d.theta = yaw;

  return pose2d;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_node");

  Slam slam_node;
  ros::Rate loop_rate(slam_node.ros_loop_rate_);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}