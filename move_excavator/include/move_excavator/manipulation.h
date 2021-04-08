/*!
 * \manipulation.h
 * \brief Manipulation Planning for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * * \date April 28, 2020
 */

#ifndef MANIPULATION_H
#define MANIPULATION_H

#include <math.h>
#include <stdio.h>
#include <eigen3/Eigen/Dense>

// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <srcp2_msgs/ExcavatorScoopMsg.h>

#include <motion_control/ArmGroup.h>
#include <move_excavator/MultiAgentState.h>
#include <move_excavator/ExcavationStatus.h>

#include <driving_tools/RotateInPlace.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>

#include <move_excavator/HomeArm.h>
#include <move_excavator/LowerArm.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/AfterScoop.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/ExcavatorFK.h>
#include <move_excavator/GoToPose.h>
#include <move_excavator/ControlInvJac.h>


#define PI 3.141592653589793
/*!
 * \def JOINT LIMITS
 */

#define JOINT1_MAX PI
#define JOINT1_MIN -PI
#define JOINT2_MAX PI/3
#define JOINT2_MIN -PI/5
#define JOINT3_MAX PI/3
#define JOINT3_MIN -PI/3
#define JOINT4_MAX 5*PI/4
#define JOINT4_MIN 0



#define STOP -1
#define HOME_MODE 0
#define LOWER_MODE 1
#define SCOOP_MODE 2
#define EXTEND_MODE 3
#define DROP_MODE 4
#define START 9


class Manipulation
{
public:
  // Constructor.
  Manipulation(ros::NodeHandle & nh);

  // Callback function for subscriber
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void bucketCallback(const srcp2_msgs::ExcavatorScoopMsg::ConstPtr &msg);

  // Callback function for subscriber.
  void goalVolatileCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void targetBinCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void manipulationStateCallback(const std_msgs::Int64::ConstPtr &msg);
  void haulerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void laserCallbackHauler(const sensor_msgs::LaserScan::ConstPtr &msg);

  // State-machine mode
  int mode = HOME_MODE;

  // Flags Init
  bool manipulation_enabled_ = false;
  bool isBucketFull_ = false;
  bool hauler_in_range_ = false;
  bool found_volatile_ = false;

  ros::Time manipulation_start_time_ = ros::Time::now();

  void executeHomeArm(double timeout);
  void executeLowerArm(double timeout);
  void executeScoop(double timeout);
  void executeAfterScoop(double timeout);
  void executeExtendArm(double timeout);
  void executeDrop(double timeout);
  void executeGoToPose(double timeout, const geometry_msgs::PoseStamped::ConstPtr &msg);
  void outputManipulationStatus();
  void getRelativePosition();
  void getForwardKinematics(double timeout);

private:
  // Node Handle
  ros::NodeHandle & nh_;

  // Publisher
  ros::Publisher pubExcavationStatus;

  // Subscriber
  // ros::Subscriber subOdometry;
  // ros::Subscriber subHaulerOdom;
  // ros::Subscriber subLaserScanHauler;
  ros::Subscriber subJointStates;
  ros::Subscriber subBucketInfo;
  ros::Subscriber subGoalVolatile;
  ros::Subscriber subTargetBin;
  ros::Subscriber subManipulationState;

  // Clients
  ros::ServiceClient clientHomeArm;
  ros::ServiceClient clientExtendArm;
  ros::ServiceClient clientLowerArm;
  ros::ServiceClient clientScoop;
  ros::ServiceClient clientAfterScoop;
  ros::ServiceClient clientDropVolatile;
  ros::ServiceClient clientFK;
  ros::ServiceClient clientGoToPose;

  // Parameters
  std::string robot_name_;
  std::string node_name_;

  // End-effector Pose Init
  geometry_msgs::PoseStamped eePose_;


  // Excavator Pose Init
  double posx_ = 0.0;
  double posy_ = 0.0;
  double posz_ = 0.0;
  double orientx_ = 0.0;
  double orienty_ = 0.0;
  double orientz_ = 0.0;
  double orientw_ = 0.0;

  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;

  // Hauler Pose Init
  double posx_hauler_ = 0.0;
  double posy_hauler_ = 0.0;
  double posz_hauler_ = 0.0;
  double orientx_hauler_ = 0.0;
  double orienty_hauler_ = 0.0;
  double orientz_hauler_ = 0.0;
  double orientw_hauler_ = 0.0;


  // Joint Positions Init
  double q1_pos_ = 0.0;
  double q2_pos_ = 0.0;
  double q3_pos_ = 0.0;
  double q4_pos_ = 0.0;

  // Bucket Info Init
  bool volatile_in_bucket_ = false;
  bool regolith_in_bucket = false;

  // Goal Volatile Pose
  geometry_msgs::PoseStamped volatile_pose_;

  // Target Bin Point  
  geometry_msgs::PointStamped bin_point_;

  // Relative Localization
  double relative_heading_ = 1.57;
  double volatile_heading_ = 0;


  // Transforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener;
  geometry_msgs::TransformStamped odom_to_base_link;
  geometry_msgs::TransformStamped base_link_to_odom;
  geometry_msgs::TransformStamped base_link_to_arm_mount;
  geometry_msgs::TransformStamped arm_mount_to_base_link;
  geometry_msgs::TransformStamped odom_to_arm_mount;
  geometry_msgs::TransformStamped arm_mount_to_odom;
  geometry_msgs::TransformStamped camera_link_to_arm_mount;
  geometry_msgs::TransformStamped arm_mount_to_camera_link;
};

#endif // MANIPULATION_H
