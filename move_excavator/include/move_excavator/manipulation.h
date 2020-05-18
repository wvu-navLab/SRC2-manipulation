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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <srcp2_msgs/ExcavatorMsg.h>
#include <motion_control/JointGroup.h>
#include <move_excavator/RotateInPlace.h>
#include <move_excavator/Stop.h>
#include <move_excavator/MoveForward.h>
#include <move_excavator/HomeArm.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/MultiAgentState.h>
#include <move_excavator/ExcavationStatus.h>

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

class Manipulation
{
public:
  // Constructor.
  Manipulation(ros::NodeHandle & nh);

  // Callback function for subscriber.
  void goalCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void bucketCallback(const srcp2_msgs::ExcavatorMsg::ConstPtr &msg);
  void alignCallback(const move_excavator::MultiAgentState::ConstPtr &msg);

private:
  // Node Handle
  ros::NodeHandle & nh_;

  // Publisher
  ros::Publisher pubFinished;

  // Subscriber
  ros::Subscriber subGoalVolatile;
  ros::Subscriber subBucketInfo;
  ros::Subscriber subMultiAgent;

  // Clients
  ros::ServiceClient clientHomeArm;
  ros::ServiceClient clientExtendArm;
  ros::ServiceClient clientDigVolatile;
  ros::ServiceClient clientScoop;
  ros::ServiceClient clientDropVolatile;
  ros::ServiceClient clientRotateInPlace;
  ros::ServiceClient clientMoveForward;
  ros::ServiceClient clientStop;

  // End effector position
  double x_goal, y_goal, z_goal, phi_goal;
  Eigen::VectorXd pos_goal = Eigen::VectorXd::Zero(3);

  bool isBucketFull;
  bool isArmHome;
  bool enableAlign;
  bool isAligned;

  double mass_collected;

  void callExtendArm();
  void callDropVolatile();
};

#endif // MANIPULATION_H