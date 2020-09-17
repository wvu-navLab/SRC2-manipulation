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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
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
#include <move_excavator/AfterScoop.h>
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



#define STOP -1
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
  void goalCallback(const geometry_msgs::Pose::ConstPtr &msg);
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

  // Bucket Info Init
  int scoop_counter_ = 0;
  double mass_collected_ = 0.0;
  double remaining_mass_thres_ = 0.1;

  void executeHomeArm(double timeout);
  void executeDig(double timeout);
  void executeScoop(double timeout);
  void executeAfterScoop(double timeout);
  void executeExtendArm(double timeout);
  void executeDrop(double timeout);
  void outputManipulationStatus();
  void getRelativePosition();

private:
  // Node Handle
  ros::NodeHandle & nh_;

  // Publisher
  ros::Publisher pubExcavationStatus;
  ros::Publisher pubMeasurementUpdate;

  // Subscriber
  ros::Subscriber subOdometry;
  ros::Subscriber subHaulerOdom;
  ros::Subscriber subJointStates;
  ros::Subscriber subBucketInfo;
  ros::Subscriber subGoalVolatile;
  ros::Subscriber subManipulationState;
  ros::Subscriber subLaserScanHauler;

  // Clients
  ros::ServiceClient clientHomeArm;
  ros::ServiceClient clientExtendArm;
  ros::ServiceClient clientDigVolatile;
  ros::ServiceClient clientScoop;
  ros::ServiceClient clientAfterScoop;
  ros::ServiceClient clientDropVolatile;

  ros::ServiceClient clientFK;

  // End-effector Pose Init
  geometry_msgs::Pose eePose_;

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

  const double LASER_THRESH = 1.5;
  const int LASER_SET_SIZE = 20;
  const int LASER_COUNTER_THRESH = 20;
  int counter_laser_collision_;

  // Joint Positions Init
  double q1_pos_ = 0.0;
  double q2_pos_ = 0.0;
  double q3_pos_ = 0.0;
  double q4_pos_ = 0.0;

  // Bucket Info Init
  double mass_in_bucket_ = 0.0;

  // Goal Volatile Pos Init
  double x_goal_ = 0.0;
  double y_goal_ = 0.0;
  double z_goal_ = 0.0;
  double orientx_goal_ = 0.0;
  double orienty_goal_ = 0.0;
  double orientz_goal_ = 0.0;
  double orientw_goal_ = 0.0;

  Eigen::VectorXd pos_goal_ = Eigen::VectorXd::Zero(3);

  int optimization_counter_ = 0;
  double volatile_heading_;
  std::vector<double> heading_options_{0.0, M_PI/12, -M_PI/12, M_PI/6, -M_PI/6};
  std::vector<double> optimization_options_{M_PI/24, -M_PI/24};
  std::vector<double> mass_optimization_{0.0, 0.0, 0.0};


  void getForwardKinematics();
  void updateLocalization();
};

#endif // MANIPULATION_H
