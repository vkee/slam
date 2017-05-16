#include "slam.h"

Slam::Slam()
{
  // Initial estimate is at the origin
  est_robot_pose_ = Eigen::Matrix4f::Identity();

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

  // init_localization();
  // ROS_INFO("Slam Node Initialized");
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
  prior_rot_stddev_ = deg_2_rad(prior_rot_stddev_);

  // Standard deviation of the translation components of the odometry measurements
  nh_.getParam("odom_trans_stddev", odom_trans_stddev_);
  // Standard deviation of the rotation components of the odometry measurements
  nh_.getParam("odom_rot_stddev", odom_rot_stddev_);
  odom_rot_stddev_ = deg_2_rad(odom_rot_stddev_);

  // Standard deviation of the translation components of the landmark observations
  nh_.getParam("land_obs_trans_stddev", land_obs_trans_stddev_);
  // Standard deviation of the rotation components of the landmark observations
  nh_.getParam("land_obs_rot_stddev", land_obs_rot_stddev_);
  land_obs_rot_stddev_ = deg_2_rad(land_obs_rot_stddev_);

  // The source frame for the tf transformLookup
  nh_.getParam("source_frame", source_frame_);
  // The target frame for the tf transformLookup
  nh_.getParam("target_frame", target_frame_);
}

void Slam::setup_subs_pubs_srvs()
{
  // // Subscriber for odometry measurements (preprocessed laser scan matcher relative poses)
  // std::string odom_meas_topic;
  // int odom_meas_queue_size;
  // if (nh_.getParam("odom_meas_topic", odom_meas_topic) && nh_.getParam("odom_meas_queue_size", odom_meas_queue_size))
  // {
  //   odom_meas_sub_ = nh_.subscribe(odom_meas_topic, odom_meas_queue_size, &Slam::odom_meas_cb, this);
  //   ROS_DEBUG("Odometry Measurement Topic %s Subscriber Loaded", odom_meas_topic.c_str());
  // }
  // else
  // {
  //   ROS_FATAL("Odometry Measurement Topic subscriber not loaded");
  // }

  // Subscriber for odometry measurements (using the estimated global poses)
  std::string delta_odom_posestamped;
  int delta_odom_posestamped_queue_size;
  if (nh_.getParam("delta_odom_posestamped", delta_odom_posestamped) && nh_.getParam("delta_odom_posestamped_queue_size", delta_odom_posestamped_queue_size))
  {
    odom_meas_global_sub_ = nh_.subscribe(delta_odom_posestamped, delta_odom_posestamped_queue_size, &Slam::robot_pose_est_cb, this);
    ROS_DEBUG("Global Odometry Measurement Topic %s Subscriber Loaded", delta_odom_posestamped.c_str());
  }
  else
  {
    ROS_FATAL("Global Odometry Measurement Topic subscriber not loaded");
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
  // tf::StampedTransform robot_base_T_cam;
  // double curr_time = ros::Time::now().toSec();

  // // Get the tf transform
  // bool success = tf_listener_.waitForTransform(target_frame_, source_frame_, ros::Time(0), ros::Duration(1.0));

  // if (success)
  // {
  //   // Get the tf transform
  //   tf_listener_.lookupTransform(target_frame_, source_frame_, ros::Time(0), robot_base_T_cam);

  //   // Getting the rotation matrix
  //   Eigen::Quaterniond eigen_quat;
  //   tf::quaternionTFToEigen(robot_base_T_cam.getRotation(), eigen_quat);
  //   robot_base_T_cam_.topLeftCorner(3,3) = eigen_quat.toRotationMatrix().cast<float>();

  //   // Getting the translation vector
  //   tf::Vector3 tf_trans = robot_base_T_cam.getOrigin();
  //   Eigen::Vector3f trans;
  //   trans(0) = tf_trans.getX();
  //   trans(1) = tf_trans.getY();
  //   trans(2) = tf_trans.getZ();
  //   robot_base_T_cam_.topRightCorner(3,1) = trans;

  //   std::cout << "translation x: " << trans(0) << " y: " << trans(1) << " z: " << trans(2) << std::endl;
 
  //   Eigen::Matrix3d rot = robot_base_T_cam_.topLeftCorner(3,3).cast<double>();

  //   // Converting to KF state vector format
  //   Eigen::Quaterniond quat(rot);
  //   tf::Quaternion quat_tf;
  //   tf::quaternionEigenToTF(quat, quat_tf);
  //   // Convert quaternion matrix to roll, pitch, yaw (in radians)
  //   double roll, pitch, yaw;
  //   tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  //   std::cout << "Roll " << roll << " Pitch " << pitch << " Yaw " << yaw << std::endl;
  // }

  // else
  // {
  //   Eigen::Vector3f trans;
  //   trans(0) = 0.371;
  //   trans(1) = 0.0;
  //   trans(2) = 0.187;
  //   robot_base_T_cam_.topRightCorner(3,1) = trans;

  //   tf::Quaternion quat_tf = tf::createQuaternionFromRPY(-1.5708, 0.0, -1.5708);

  //   Eigen::Quaterniond eigen_quat;
  //   tf::quaternionTFToEigen(tf_quat, eigen_quat);
  //   robot_base_T_cam_.topLeftCorner(3,3) = eigen_quat.toRotationMatrix().cast<float>();
  // }

  // Hardcoded values 
  Eigen::Vector3f trans;
  trans(0) = 0.371;
  trans(1) = 0.0;
  trans(2) = 0.187;
  robot_base_T_cam_.topRightCorner(3,1) = trans;

  tf::Quaternion quat_tf = tf::createQuaternionFromRPY(-1.5708, 0.0, -1.5708);

  Eigen::Quaterniond eigen_quat;
  tf::quaternionTFToEigen(quat_tf, eigen_quat);
  robot_base_T_cam_.topLeftCorner(3,3) = eigen_quat.toRotationMatrix().cast<float>();

  return true;
}

// Initializes the GTSAM localization
void Slam::init_localization()
{
  localization_.init_localization(isam2_relinearize_thresh_, isam2_relinearize_skip_, 
    prior_trans_stddev_, prior_rot_stddev_, odom_trans_stddev_, odom_rot_stddev_,
    land_obs_trans_stddev_, land_obs_rot_stddev_);
}

// Callback for receiving the current estimated robot pose and adding the odometry measurement 
void Slam::robot_pose_est_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped est_robot_delta_msg = *msg;

  // Getting the latest pose estimate
  Eigen::Matrix4f delta_robot_pose = pose_msg_2_transform(est_robot_delta_msg.pose);
  slam::Localization::Pose2D delta_robot_pose_2d = transform_2_pose2d(delta_robot_pose);

  // Copy the estimated state
  Eigen::Matrix4f est_robot_pose_copy = est_robot_pose_;

  // Updating the current estimated robot pose
  est_robot_pose_ = est_robot_pose_ * delta_robot_pose;

  // Updating the translation measurements manually
  Eigen::Vector3d est_trans = est_robot_pose_copy.topRightCorner(3,1).cast<double>();
  est_trans(0) = est_trans(0) + delta_robot_pose_2d.x;
  est_trans(1) = est_trans(1) + delta_robot_pose_2d.y;
  est_robot_pose_.topRightCorner(3,1) = est_trans.cast<float>();

  slam::Localization::Pose2D  est_robot_pose2d = transform_2_pose2d(est_robot_pose_);

  // Adding the odometry measurement to the factor graph
  localization_.add_odom_measurement(delta_robot_pose_2d.x, delta_robot_pose_2d.y, delta_robot_pose_2d.theta,
    est_robot_pose2d.x, est_robot_pose2d.y, est_robot_pose2d.theta);

  // Publish pose estimate
  est_robot_pose_pub_.publish(pose2d_2_pose_stamped_msg(est_robot_pose2d));
}

// Callback for receiving the odometry msg
void Slam::odom_meas_cb(const geometry_msgs::Pose2DConstPtr& msg)
{
  // geometry_msgs::Pose2D odom_meas_msg = *msg;
  // std::cout << "odom_meas_msg: " << odom_meas_msg << std::endl;
  // // Adding the odometry measurement to the factor graph and optimizing the graph
  // slam::Localization::Pose2D est_robot_pose = localization_.add_odom_measurement(odom_meas_msg.x, odom_meas_msg.y, odom_meas_msg.theta);

  // // Publish pose estimate
  // est_robot_pose_pub_.publish(pose2d_2_pose_stamped_msg(est_robot_pose));
}

// Callback for receiving the landmark measurement msg
void Slam::land_meas_cb(const apriltags_ros::AprilTagDetectionArrayConstPtr& msg)
{
  apriltags_ros::AprilTagDetectionArray detections_array_msg = *msg;
  std::vector<apriltags_ros::AprilTagDetection> detections = detections_array_msg.detections;

  std::cout << "detections.size(): " << detections.size() << std::endl;

  bool can_optimize_graph = false;

  for (int i = 0; i < detections.size(); i++)
  {
    apriltags_ros::AprilTagDetection indiv_detection = detections[i];

    int landmark_id = indiv_detection.id;
    std::cout << "landmark_id: " << landmark_id << std::endl;
    geometry_msgs::PoseStamped pose = indiv_detection.pose;

    std::string april_tag_frame_id = "april_tag_frame_id_";
    april_tag_frame_id = april_tag_frame_id + std::to_string(landmark_id);

    // Computing the transform from the robot base_link to the landmark
    Eigen::Matrix4f cam_T_landmark = pose_msg_2_transform(pose.pose);
    Eigen::Matrix4f robot_base_T_landmark = robot_base_T_cam_ * cam_T_landmark;
    // Convert the transform to a Pose2D
    slam::Localization::Pose2D landmark_meas = transform_2_pose2d(robot_base_T_landmark);
    std::cout << "Eigen: base_footprint_2_landmark x: " << landmark_meas.x << " y: " << landmark_meas.y << " theta: " << landmark_meas.theta << std::endl;

    // Pose of landmark in world frame
    Eigen::Matrix4f world_T_landmark = est_robot_pose_ * robot_base_T_landmark;
    slam::Localization::Pose2D world_landmark_meas = transform_2_pose2d(world_T_landmark);
    std::cout << "Eigen: odom_2_landmark x: " << world_landmark_meas.x << " y: " << world_landmark_meas.y << " theta: " << world_landmark_meas.theta << std::endl;

    // Looking up transform using TF
    tf::StampedTransform robot_base_T_tag;
    double curr_time = ros::Time::now().toSec();
    // Get the tf transform
    bool success = tf_listener_.waitForTransform(target_frame_, april_tag_frame_id, ros::Time(0), ros::Duration(1.0));
    if (success)
    {
      // Get the tf transform
      tf_listener_.lookupTransform(target_frame_, april_tag_frame_id, ros::Time(0), robot_base_T_tag);

      // Getting the rotation matrix
      Eigen::Quaterniond eigen_quat;
      tf::quaternionTFToEigen(robot_base_T_tag.getRotation(), eigen_quat);  
      Eigen::Matrix3d rot = eigen_quat.toRotationMatrix().cast<double>();

      // Converting to KF state vector format
      Eigen::Quaterniond quat(rot);
      tf::Quaternion quat_tf;
      tf::quaternionEigenToTF(quat, quat_tf);
      // Convert quaternion matrix to roll, pitch, yaw (in radians)
      double roll, pitch, yaw;
      tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
      roll = rad_2_deg(roll);
      pitch = rad_2_deg(pitch);
      yaw = rad_2_deg(yaw);

      std::cout << "TF: base_footprint_2_landmark Roll " << roll << " Pitch " << pitch << " Yaw " << yaw << std::endl;

      // Getting the translation vector
      tf::Vector3 tf_trans = robot_base_T_tag.getOrigin();
      Eigen::Vector3f trans;
      trans(0) = tf_trans.getX();
      trans(1) = tf_trans.getY();
      trans(2) = tf_trans.getZ();

      std::cout << "TF: base_footprint_2_landmark x: " << tf_trans.getX() << " y: " << tf_trans.getY() << " z: " << tf_trans.getZ() << std::endl;
      double theta = atan2(tf_trans.getY(), tf_trans.getX());
      double test_theta = rad_2_deg(theta);
      std::cout << "TF ATAN Theta: " << test_theta << std::endl;

      double x = tf_trans.getX();
      double y = tf_trans.getY();

      slam::Localization::Pose2D est_rob_2d = transform_2_pose2d(est_robot_pose_);
      double est_rob_x = est_rob_2d.x;
      double est_rob_y = est_rob_2d.y;

      double manual_x = est_rob_x + x;
      double manual_y = est_rob_y + y;
      std::cout << "Eigen: odom_2_landmark x: " << manual_x << " y: " << manual_y << std::endl;

      // NOW LOOKING UP FROM ODOM TO THE APRIL TAG
      // Looking up transform using TF
      tf::StampedTransform odom_2_tag;
      double curr_time = ros::Time::now().toSec();
      // Get the tf transform
      bool success2 = tf_listener_.waitForTransform("odom", april_tag_frame_id, ros::Time(0), ros::Duration(1.0));
      if (success2)
      {
        // Get the tf transform
        tf_listener_.lookupTransform("odom", april_tag_frame_id, ros::Time(0), odom_2_tag);

        // Getting the rotation matrix
        Eigen::Quaterniond eigen_quat2;
        tf::quaternionTFToEigen(odom_2_tag.getRotation(), eigen_quat2);  
        Eigen::Matrix3d rot2 = eigen_quat2.toRotationMatrix().cast<double>();

        // Converting to KF state vector format
        Eigen::Quaterniond quat2(rot2);
        tf::Quaternion quat_tf2;
        tf::quaternionEigenToTF(quat2, quat_tf2);
        // Convert quaternion matrix to roll, pitch, yaw (in radians)
        double roll2, pitch2, yaw2;
        tf::Matrix3x3(quat_tf2).getRPY(roll2, pitch2, yaw2);
        roll2 = rad_2_deg(roll2);
        pitch2 = rad_2_deg(pitch2);
        yaw2 = rad_2_deg(yaw2);

        std::cout << "TF: odom_2_tag Roll " << roll2 << " Pitch " << pitch2 << " Yaw " << yaw2 << std::endl;

        tf::Vector3 tf_trans2 = robot_base_T_tag.getOrigin();

        std::cout << "TF: odom_2_tag x: " << tf_trans2.getX() << " y: " << tf_trans2.getY() << " z: " << tf_trans2.getZ() << std::endl;
        double theta2 = atan2(tf_trans2.getY(), tf_trans2.getX());
        double test_theta2 = rad_2_deg(theta);
        std::cout << "TF ATAN Theta: " << test_theta2 << std::endl;

        // Updating the factor graph
        // Adds/stores a landmark measurement to iSAM2 and returns whether the factor graph can be optimized
        bool optimize_graph = localization_.add_landmark_measurement(landmark_id, landmark_meas.x, landmark_meas.y, theta,
          tf_trans2.getX(), tf_trans2.getY(), theta2);

        // If any of the landmark measurements enable optimization to happen, then denote that
        if (optimize_graph)
        {
          can_optimize_graph = true;
        }
      }
    }
  }

  // Only optimize graph if safe to do so
  if (can_optimize_graph)
  {
    // Optimize the factor graph
    localization_.optimize_factor_graph();

    // Get the estimated robot pose
    slam::Localization::Pose2D est_robot_pose = localization_.get_est_robot_pose();

    // Update estimated robot pose
    est_robot_pose_ = pose2d_2_transform(est_robot_pose);

    // Publish pose estimate
    est_robot_pose_pub_.publish(pose2d_2_pose_stamped_msg(est_robot_pose));
  }
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

// Converts a Pose2D struct into an Eigen transform
Eigen::Matrix4f Slam::pose2d_2_transform(slam::Localization::Pose2D pose2d)
{
  Eigen::Matrix4f transform;

  tf::Quaternion tf_quat = tf::createQuaternionFromYaw(pose2d.theta);
  Eigen::Quaterniond quat;
  tf::quaternionTFToEigen(tf_quat, quat);
  transform.topLeftCorner(3,3) = quat.toRotationMatrix().cast<float>();

  // Updating the translation measurements
  Eigen::Vector3f trans;
  trans(0) = pose2d.x;
  trans(1) = pose2d.y;
  trans(2) = 0.0;
  transform.topRightCorner(3,1) = trans;

  return transform;
}

// Converts degrees to radians
double Slam::deg_2_rad(double input)
{
  return input*M_PI/180.0;
}

// Converts radians to degrees
double Slam::rad_2_deg(double input)
{
  return input*180.0/M_PI;
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