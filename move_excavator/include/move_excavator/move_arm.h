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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/ArmGroup.h>
#include <move_excavator/HomeArm.h>
#include <move_excavator/ExtendArm.h>
#include <move_excavator/DropVolatile.h>
#include <move_excavator/Scoop.h>
#include <move_excavator/AfterScoop.h>
#include <move_excavator/LowerArm.h>
#include <move_excavator/RetractArm.h>
#include <move_excavator/ExcavatorFK.h>
#include <move_excavator/GoToPose.h>
#include <move_excavator/ControlInvJac.h>

#define PI 3.141592653589793

/*!
 * \def JOINT LIMITS
 */
#define JOINT1_MAX PI
#define JOINT1_MIN -PI
#define JOINT2_MAX 3*PI/8
#define JOINT2_MIN -3*PI/8
#define JOINT3_MAX PI/4
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
  bool RetractArm(move_excavator::RetractArm::Request  &req, move_excavator::RetractArm::Response &res);
  bool ExcavatorFK(move_excavator::ExcavatorFK::Request &req, move_excavator::ExcavatorFK::Response &res);
  bool GoToPose(move_excavator::GoToPose::Request &req, move_excavator::GoToPose::Response &res);
  bool ControlInvJac(move_excavator::ControlInvJac::Request &req, move_excavator::ControlInvJac::Response &res);

private:
  // Node Handle
  ros::NodeHandle & nh_;

  // Publishers
  ros::Publisher pubJointAngles;

  // Subscribers
  ros::Subscriber subJointStates;

  // Service Servers
  ros::ServiceServer serverHomeArm;
  ros::ServiceServer serverExtendArm;
  ros::ServiceServer serverDropVolatile;
  ros::ServiceServer serverScoop;
  ros::ServiceServer serverAfterScoop;
  ros::ServiceServer serverLowerArm;
  ros::ServiceServer serverRetractArm;
  ros::ServiceServer serverFK;
  ros::ServiceServer serverGoToPose;
  ros::ServiceServer serverControlInvJac;

  // Namespaces
  std::string node_name_; 
  std::string robot_name_; 

  // Link lengths
  double a0_, d0_;
  double a1_, d1_, alpha1_;
  double a2_;
  double a3_;
  double a4_, d4_;

  // Joint Positions
  double q1_curr_ = 0.0;
  double q2_curr_ = 0.0;
  double q3_curr_ = 0.0;
  double q4_curr_ = 0.0;
  
  // DH Parameters 
  Eigen::VectorXd a_DH_ = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd alpha_DH_ = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd d_DH_ = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd theta_DH_ = Eigen::VectorXd::Zero(5);

  // Homogeneous transform arm mount to end-effector
  Eigen::MatrixXd T0Tn_ = Eigen::MatrixXd::Identity(4,4);

  // Inverse Jacobian Control Parameters
  double max_pos_error_;
  double dt_;
  double K_pos_;

  // Util Functions
  std::vector<double> LinearSpacedArray(double a, double b, std::size_t N);

  // Geometric Functions
  void ConstrainAngle(double& q);
  void LimitJoint(double& q, double max, double min);

  // Transformation Functions
  Eigen::MatrixXd Trotx(double alpha);
  Eigen::MatrixXd Trotz(double theta);
  Eigen::MatrixXd Transl(double x, double y, double z);

  // Kinematic Functions
  geometry_msgs::PoseStamped SolveFK(double q1, double q2, double q3, double q4);

  // Inverse Kinematics Functions  
  std::pair<bool, Eigen::VectorXd> SolveIK(Eigen::VectorXd goal_xyz);
  inline double RPolyFit(double x, double y);
  inline double ZPolyFit(double x, double y);

  // Control Functions
  Eigen::MatrixXd Jtraj(Eigen::VectorXd q0, Eigen::VectorXd q1, int steps);
  Eigen::MatrixXd CalculateJacobian();
  Eigen::MatrixXd InvertJacobian(Eigen::MatrixXd J);

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
  geometry_msgs::TransformStamped transform_to_arm_mount;

  // Target Point  
  geometry_msgs::PointStamped goal_point_;


  // Polyfit coeffs
  double r00 =       2.009;  //(2.009, 2.009)
  double r10 =   4.812e-15;  //(-5.174e-05, 5.174e-05)
  double r01 =  -3.027e-16;  //(-5.174e-05, 5.174e-05)
  double r20 =     -0.7937;  //(-0.7938, -0.7936)
  double r11 =     -0.5267;  //(-0.5267, -0.5266)
  double r02 =     -0.1748;  //(-0.1749, -0.1747)
  double r30 =  -3.086e-15;  //(-5.097e-05, 5.097e-05)
  double r21 =  -4.278e-16;  //(-4.475e-05, 4.475e-05)
  double r12 =  -2.298e-16;  //(-4.475e-05, 4.475e-05)
  double r03 =    4.48e-16;  //(-5.097e-05, 5.097e-05)
  double r40 =     0.05932;  //(0.05923, 0.0594)
  double r31 =     0.07707;  //(0.07699, 0.07714)
  double r22 =      0.0767;  //(0.07663, 0.07678)
  double r13 =     0.03301;  //(0.03294, 0.03309)
  double r04 =    0.004992;  //(0.004907, 0.005077)

  double z00 =        0.12;  //(0.1198, 0.1202)
  double z10 =      -1.582;  //(-1.583, -1.582)
  double z01 =     -0.5224;  //(-0.5227, -0.5221)
  double z20 =   3.552e-16;  //(-0.000706, 0.000706)
  double z11 =   5.092e-16;  //(-0.0005825, 0.0005825)
  double z02 =  -1.241e-16;  //(-0.000706, 0.000706)
  double z30 =      0.2338;  //(0.2335, 0.2342)
  double z21 =      0.2259;  //(0.2257, 0.2262)
  double z12 =      0.1471;  //(0.1468, 0.1473)
  double z03 =     0.02976;  //(0.02944, 0.03008)
  double z40 =  -1.397e-15;  //(-0.0005312, 0.0005312)
  double z31 =  -3.808e-16;  //(-0.0004634, 0.0004634)
  double z22 =  -2.867e-16;  //(-0.0004549, 0.0004549)
  double z13 =  -1.396e-16;  //(-0.0004634, 0.0004634)
  double z04 =   1.901e-16;  //(-0.0005312, 0.0005312)

};

#endif // MOVE_ARM_H
