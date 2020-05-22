/*!
 * \manipulation.h
 * \brief Manipulation Planning for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <srcp2_msgs/ExcavatorMsg.h>

#include <motion_control/JointGroup.h>
#include <move_excavator/MultiAgentState.h>
#include <move_excavator/ExcavationStatus.h>

#include <driving_tools/RotateInPlace.h>
#include <driving_tools/Stop.h>
#include <driving_tools/MoveForward.h>

#include <move_excavator/HomeArm.h>
#include <move_excavator/DigVolatile.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/ExcavatorFK.h>

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



#define HOME_MODE 0
#define DIG_MODE 1
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
  void bucketCallback(const srcp2_msgs::ExcavatorMsg::ConstPtr &msg);

  // Callback function for subscriber.
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void debugCallback(const std_msgs::Int64::ConstPtr &msg);
  void haulerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  // State-machine mode
  int mode = HOME_MODE;

  // Flags Init
  bool isManipulationStep_ = false;
  bool isBucketFull_ = false;
  bool isHaulerInRange_ = false;

  void executeHomeArm();
  void executeDig();
  void executeScoop();
  void executeExtendArm();
  void executeDrop();

private:
  // Node Handle
  ros::NodeHandle & nh_;

  // Publisher
  ros::Publisher pubFinished;
  ros::Publisher pubOdomFromVolatile;

  // Subscriber
  ros::Subscriber subOdometry;
  ros::Subscriber subHaulerOdom;
  ros::Subscriber subJointStates;
  ros::Subscriber subBucketInfo;
  ros::Subscriber subGoalVolatile;
  ros::Subscriber subDebug;

  // Clients
  ros::ServiceClient clientHomeArm;
  ros::ServiceClient clientExtendArm;
  ros::ServiceClient clientDigVolatile;
  ros::ServiceClient clientScoop;
  ros::ServiceClient clientDropVolatile;

  ros::ServiceClient clientFK;

  // End-effector Pose Init
  geometry_msgs::PoseStamped eePose; 

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

  double relative_heading_;

  // Joint Positions Init
  double q1_pos_ = 0.0;
  double q2_pos_ = 0.0;
  double q3_pos_ = 0.0;
  double q4_pos_ = 0.0;

  // Bucket Info Init
  double mass_in_bucket_ = 0.0;
  double mass_collected_ = 0.0;

  // Goal Volatile Pos Init
  double x_goal_ = 0.0;
  double y_goal_ = 0.0;
  double z_goal_ = 0.0;
  double orientx_goal_ = 0.0;
  double orienty_goal_ = 0.0;
  double orientz_goal_ = 0.0;
  double orientw_goal_ = 0.0;
  
  Eigen::VectorXd pos_goal_ = Eigen::VectorXd::Zero(3);

  // bool isArmHome = false;
  // bool enableAlign = false;
  // bool isAligned = false;

  void getForwardKinematics();
  void updateLocalization();
};

#endif // MANIPULATION_H