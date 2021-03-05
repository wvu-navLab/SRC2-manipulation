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

  // Callback function for subscriber
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

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
  double a0_, d0_;
  double a1_, d1_, alpha1_;
  double a2_;
  double a3_;
  double a4_;

  // Joint Positions
  double q1_curr_ = 0.0;
  double q2_curr_ = 0.0;
  double q3_curr_ = 0.0;
  double q4_curr_ = 0.0;

  Eigen::VectorXd a_DH_ = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd alpha_DH_ = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd d_DH_ = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd theta_DH_ = Eigen::VectorXd::Zero(5);

  // Geometric Functions
  void constrainAngle(double& q);
  void limitJoint(double& q, double max, double min);

  // Transformation Functions
  Eigen::MatrixXd trotx(double alpha);
  Eigen::MatrixXd trotz(double theta);
  Eigen::MatrixXd transl(double x, double y, double z);

  // Kinematic Functions
  geometry_msgs::Pose calculateFK(double q1, double q2, double q3, double q4);

  Eigen::MatrixXd T0Tn_ = Eigen::MatrixXd::Identity(4,4);

  // Inverse Kinematics Functions  
  Eigen::VectorXd calculateIK(Eigen::VectorXd goal);

  Eigen::VectorXd pos_goal_ = Eigen::VectorXd::Zero(3);

  // Control Functions
  Eigen::MatrixXd jtraj(Eigen::VectorXd q0, Eigen::VectorXd q1, int steps);
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd invertJacobian(Eigen::MatrixXd J);

  std::string node_name_; 
  double max_pos_error_;
  double max_q4_error_;
  double dt_;
  double K_pos_;
  double K_q4_;

};

#endif // MOVE_ARM_H
