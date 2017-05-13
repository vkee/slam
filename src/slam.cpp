#include "slam.h"

Slam::Slam()
{
  // Get params
  get_params();

  // Setup the subscribers and publishers
  setup_subs_pubs_srvs();

  // Lookup camera transform
  bool success = lookup_camera_transform();

  if (success)
  {
    // Initialize the GTSAM library
    init_localization();

    ROS_INFO("Slam Node Initialized");
  }

  else
  {
    ROS_ERROR("Unable to find camera transform to base link, Slam node NOT initialized");
  }
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

  // The source frame for the tf transformLookup
  nh_.getParam("source_frame", source_frame_);
  // The target frame for the tf transformLookup
  nh_.getParam("target_frame", target_frame_);
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

  // Publisher of estimated robot pose
  std::string est_robot_pose_topic;
  int est_robot_queue_size;
  if (nh_.getParam("est_robot_pose_topic", est_robot_pose_topic) && nh_.getParam("est_robot_queue_size", est_robot_queue_size))
  {
    est_robot_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(est_robot_pose_topic, est_robot_queue_size, true);
  }
  else
  {
    ROS_FATAL("Estimated Robot Pose publisher not loaded");
  }
}

// Looks up transform from camera to the robot base link 
// Returns true if successful
bool Slam::lookup_camera_transform()
{
  tf::StampedTransform robot_base_T_cam;
  double curr_time = ros::Time::now().toSec();

  // Get the tf transform
  bool success = tf_listener_.waitForTransform(target_frame_, source_frame_, ros::Time(0), ros::Duration(5.0));

  if (success)
  {
    // Get the tf transform
    tf_listener_.lookupTransform(target_frame_, source_frame_, ros::Time(0), robot_base_T_cam);

    // Getting the rotation matrix
    Eigen::Quaterniond eigen_quat;
    tf::quaternionTFToEigen(robot_base_T_cam.getRotation(), eigen_quat);
    robot_base_T_cam_.topLeftCorner(3,3) = eigen_quat.toRotationMatrix().cast<float>();

    // Getting the translation vector
    tf::Vector3 tf_trans = robot_base_T_cam.getOrigin();
    Eigen::Vector3f trans;
    trans(0) = tf_trans.getX();
    trans(1) = tf_trans.getY();
    trans(2) = tf_trans.getZ();
    robot_base_T_cam_.topRightCorner(3,1) = trans;
  }

  return success;
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
  // NOTE: if too slow, to do this whole optimize, could probably just concatenate the odom on top of the est state to get new est robot pose state

  geometry_msgs::Pose2D odom_meas_msg = *msg;
  // Adding the odometry measurement to the factor graph and optimizing the graph
  slam::Localization::Pose2D est_robot_pose = localization_.add_odom_measurement(odom_meas_msg.x, odom_meas_msg.y, odom_meas_msg.theta);

  // Publish pose estimate
  est_robot_pose_pub_.publish(pose2d_2_pose_stamped_msg(est_robot_pose));
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

    // Computing the transform from the robot base_link to the landmark
    Eigen::Matrix4f cam_T_landmark = pose_msg_2_transform(pose.pose);
    Eigen::Matrix4f robot_base_T_landmark = robot_base_T_cam_ * cam_T_landmark;

    // Convert the transform to a Pose2D
    slam::Localization::Pose2D landmark_meas = transform_2_pose2d(robot_base_T_landmark);

    // Updating the factor graph
    localization_.add_landmark_measurement(landmark_id, landmark_meas.x, landmark_meas.y, landmark_meas.theta);
  }

  // Optimize the factor graph
  localization_.optimize_factor_graph();

  // Get the estimated robot pose
  slam::Localization::Pose2D est_robot_pose = localization_.get_est_robot_pose();

  // Publish pose estimate
  est_robot_pose_pub_.publish(pose2d_2_pose_stamped_msg(est_robot_pose));
}

// Converts a ROS Geometry Pose msg into a Pose2D struct (flattens the 6DoF pose to Pose2D)
slam::Localization::Pose2D Slam::transform_2_pose2d(Eigen::Matrix4f transform)
{
  Eigen::Matrix3d rot = transform.topLeftCorner(3,3).cast<double>();

  // Converting to KF state vector format
  Eigen::Quaterniond quat(rot);
  tf::Quaternion quat_tf;
  tf::quaternionEigenToTF(quat, quat_tf);
  // Convert quaternion matrix to roll, pitch, yaw (in radians)
  double roll, pitch, yaw;
  tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  // Updating the translation measurements
  Eigen::Vector3f trans = transform.topRightCorner(3,1);

  slam::Localization::Pose2D pose2d;
  pose2d.x = trans(0);
  pose2d.y = trans(1);
  pose2d.theta = yaw;

  return pose2d;
}

// Converts a ROS Geometry Pose msg into a Pose2D struct (flattens the 6DoF pose to Pose2D)
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

// Converts a Pose2D struct into a ROS Geometry Pose msg
geometry_msgs::PoseStamped Slam::pose2d_2_pose_stamped_msg(slam::Localization::Pose2D pose2d)
{
  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.frame_id = frame_id_;

  // Convert landmark to transform
  geometry_msgs::Point position;
  position.x = pose2d.x;
  position.y = pose2d.y;
  position.z = 0.0;
  tf::Quaternion quat = tf::createQuaternionFromYaw(pose2d.theta);
  geometry_msgs::Quaternion orientation;
  tf::quaternionTFToMsg(quat, orientation);
  pose_stamped_msg.pose.position = position;
  pose_stamped_msg.pose.orientation = orientation;

  return pose_stamped_msg;
}

// Converts a ROS Geometry Pose msg into an Eigen transform
Eigen::Matrix4f Slam::pose_msg_2_transform(geometry_msgs::Pose pose_msg)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Convert landmark  to transform
  geometry_msgs::Point position = pose_msg.position;
  geometry_msgs::Quaternion quat = pose_msg.orientation;

  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);
  Eigen::Quaterniond eigen_quat;
  tf::quaternionTFToEigen(tf_quat, eigen_quat);
  transform.topLeftCorner(3,3) = eigen_quat.toRotationMatrix().cast<float>();

  Eigen::Vector3f trans;
  trans(0) = position.x;
  trans(1) = position.y;
  trans(2) = position.z;
  transform.topRightCorner(3,1) = trans;

  return transform;
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