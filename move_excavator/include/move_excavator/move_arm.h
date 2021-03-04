/*!
 * \move_arm.h
 * \brief Manipulation Control for SRC2 Rovers
 *
 * Move Arm creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * * \date April 28, 2020
 */

#ifndef MOVE_ARM_H
#define MOVE_ARM_H

#include <math.h>
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <tf2/convert.h>

// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/ArmGroup.h>
#include <move_excavator/HomeArm.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/AfterScoop.h>
#include <move_excavator/LowerArm.h>
#include <move_excavator/ExcavatorFK.h>

#define PI 3.141592653589793

/*!
 * \def JOINT LIMITS
 */
#define JOINT1_MAX PI
#define JOINT1_MIN -PI
#define JOINT2_MAX 3*PI/8
#define JOINT2_MIN -3*PI/8
#define JOINT3_MAX 7*PI/8
#define JOINT3_MIN -PI/4
#define JOINT4_MAX 2*PI/3
#define JOINT4_MIN -2*PI/3

class MoveArm
{
public:
  // Constructor
  MoveArm(ros::NodeHandle & nh);

  // Services
  bool HomeArm(move_excavator::HomeArm::Request  &req, move_excavator::HomeArm::Response &res);
  bool ExtendArm(move_excavator::ExtendArm::Request  &req, move_excavator::ExtendArm::Response &res);
  bool DropVolatile(move_excavator::DropVolatile::Request  &req, move_excavator::DropVolatile::Response &res);
  bool Scoop(move_excavator::Scoop::Request  &req, move_excavator::Scoop::Response &res);
  bool AfterScoop(move_excavator::AfterScoop::Request  &req, move_excavator::AfterScoop::Response &res);
  bool LowerArm(move_excavator::LowerArm::Request  &req, move_excavator::LowerArm::Response &res);
  bool ExcavatorFK(move_excavator::ExcavatorFK::Request &req, move_excavator::ExcavatorFK::Response &res);


private:
  // Node Handle
  ros::NodeHandle & nh_;

  // Publishers
  ros::Publisher pubJointAngles;

  // Service Servers
  ros::ServiceServer serverHomeArm;
  ros::ServiceServer serverExtendArm;
  ros::ServiceServer serverDropVolatile;
  ros::ServiceServer serverScoop;
  ros::ServiceServer serverAfterScoop;
  ros::ServiceServer serverLowerArm;
  ros::ServiceServer serverFK;

  // Message
  motion_control::ArmGroup q;

  // Link lengths
  double a0, d0;
  double a1, d1;
  double a2;
  double a3;
  double a4;

  // double max_pos_error = 0.05;
  // double max_q4_error = 0.01;
  // double dt = 0.02;
  // double K_pos = 30;
  // double K_q4 = 0.2;

  // End effector position
  // double x_goal, y_goal, z_goal, phi_goal;

  // Joint angles
  // double q1_goal = 0.0;
  // double q2_goal = 0.0;
  // double q3_goal = 0.0;
  // double q4_goal = 0.0;

  Eigen::VectorXd a_DH = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd alpha_DH = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd d_DH = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd theta_DH = Eigen::VectorXd::Zero(5);

  Eigen::VectorXd pos_goal = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd T0Tn = Eigen::MatrixXd::Identity(4,4);
  // Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,4);
  // Eigen::MatrixXd pinvJ = Eigen::MatrixXd::Zero(4,6);

  // Transformation Functions
  Eigen::MatrixXd trotx(double alpha);
  Eigen::MatrixXd trotz(double theta);
  Eigen::MatrixXd transl(double x, double y, double z);

  // Kinematic Functions
  geometry_msgs::Pose calculateFK(double q1, double q2, double q3, double q4);

  // Flags
  // bool armControlEnabled_ = false;

  // Geometric Functions
  void constrainAngle(double& q_);
  void limitJoint(double& q_, double max_, double min_);
  Eigen::MatrixXd jtraj(Eigen::VectorXd q0, Eigen::VectorXd q1, int steps);
};

#endif // MOVE_ARM_H
