/*!
 * \move_arm.cpp
 * \brief Move Arm Methods for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date May 04, 2020
 */

#include "move_excavator/move_arm.h"

MoveArm::MoveArm(ros::NodeHandle & nh)
: nh_(nh),
  tf2_listener(tf_buffer)
{
  // Node publishes individual joint positions
  pubJointAngles = nh_.advertise<motion_control::ArmGroup>("control/arm/joint_angles", 1);

  // Subscriber to joint states
  subJointStates = nh_.subscribe("joint_states", 1, &MoveArm::jointStateCallback, this);

  // Service Servers
  serverHomeArm = nh_.advertiseService("manipulation/home_arm", &MoveArm::HomeArm, this);
  serverExtendArm = nh_.advertiseService("manipulation/extend_arm", &MoveArm::ExtendArm, this);
  serverDropVolatile = nh_.advertiseService("manipulation/drop_volatile", &MoveArm::DropVolatile, this);
  serverLowerArm = nh_.advertiseService("manipulation/lower_arm", &MoveArm::LowerArm, this);
  serverScoop = nh_.advertiseService("manipulation/scoop", &MoveArm::Scoop, this);
  serverAfterScoop = nh_.advertiseService("manipulation/after_scoop", &MoveArm::AfterScoop, this);
  serverGoToPose = nh_.advertiseService("manipulation/go_to_pose", &MoveArm::GoToPose, this);
  serverControlInvJac = nh_.advertiseService("manipulation/control_inv_jac", &MoveArm::GoToPose, this);
  serverFK = nh_.advertiseService("manipulation/excavator_fk", &MoveArm::ExcavatorFK, this);

  // Link lengths
  a0_ = 0.70;
  d0_ = 0.10;

  a1_ = 0.19;
  d1_ = 0.12;
  alpha1_ = -PI/2;

  a2_ = 0.80;

  a3_ = 0.80;

  a4_ = 0.23;

  // Denavit-Hartenberg Table
  a_DH_ << a0_, a1_, a2_, a3_, a4_;
  alpha_DH_ << 0, alpha1_, 0, 0, 0;
  d_DH_ << d0_, d1_, 0, 0, 0.05;
  theta_DH_ << 0, 0, 0, 0, 0;

  node_name_ ="move_arm";
  // Read params from yaml file
  if (ros::param::get(node_name_ + "/robot_name", robot_name_) == false)
  {
      ROS_FATAL("No parameter 'robot_name_' specified");
      ros::shutdown();
      exit(1);
  }
  if (ros::param::get(node_name_ + "/max_pos_error", max_pos_error_) == false)
  {
      ROS_FATAL("No parameter 'max_pos_error' specified");
      ros::shutdown();
      exit(1);
  }
  if(ros::param::get(node_name_+"/max_q4_error",max_q4_error_)==false)
  {
    ROS_FATAL("No parameter 'max_q4_error' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/dt",dt_)==false)
  {
    ROS_FATAL("No parameter 'dt' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/K_pos",K_pos_)==false)
  {
    ROS_FATAL("No parameter 'K_pos' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/K_q4",K_q4_)==false)
  {
    ROS_FATAL("No parameter 'K_q4' specified");
    ros::shutdown();
    exit(1);
  }
}

void MoveArm::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  int shoulder_yaw_joint_idx;
  int shoulder_pitch_joint_idx;
  int elbow_pitch_joint_idx;
  int wrist_pitch_joint_idx;

  // loop joint states
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "shoulder_yaw_joint") {
      shoulder_yaw_joint_idx = i;
    }
    if (msg->name[i] == "shoulder_pitch_joint") {
      shoulder_pitch_joint_idx = i;
    }
    if (msg->name[i] == "elbow_pitch_joint") {
      elbow_pitch_joint_idx = i;
    }
    if (msg->name[i] == "wrist_pitch_joint") {
      wrist_pitch_joint_idx = i;
    }
  }

  q1_curr_ = msg->position[shoulder_yaw_joint_idx];  // TODO: Get ids from the message names
  q2_curr_ = msg->position[shoulder_pitch_joint_idx];
  q3_curr_ = msg->position[elbow_pitch_joint_idx];
  q4_curr_ = msg->position[wrist_pitch_joint_idx];

  theta_DH_ << 0, q1_curr_, q2_curr_, q3_curr_, q4_curr_;
}

/*--------------------------------------------------------------------
 * -------------------- GEOMETRIC FUNCTIONS --------------------------
 *------------------------------------------------------------------*/

void MoveArm::constrainAngle(double& q)
{
    q = fmod(q + PI,2*PI);
    if (q < 0)
        q += 2*PI;
    q =  q - PI;
}

void MoveArm::limitJoint(double& q, double max, double min)
{
  (q > max)? max: q;
  (q < min)? min: q;
}

Eigen::MatrixXd MoveArm::trotx(double alpha)
{
  Eigen::MatrixXd T(4,4);
  T <<  1, 0,           0,            0,
        0, cos(alpha),  -sin(alpha),  0,
        0, sin(alpha),  cos(alpha),   0,
        0, 0,           0,            1;
  return T;
}

Eigen::MatrixXd MoveArm::trotz(double theta)
{
  Eigen::MatrixXd T(4,4);
  T <<  cos(theta), -sin(theta),  0,  0,
        sin(theta), cos(theta),   0,  0,
        0,          0,            1,  0,
        0,          0,            0,  1;
  return T;
}

Eigen::MatrixXd MoveArm::transl(double x, double y, double z)
{
  Eigen::MatrixXd T(4,4);
  T <<  1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
  return T;
}

/*--------------------------------------------------------------------
 * ------------------- JOINT TRAJECTORY FUNCTION ---------------------
 *------------------------------------------------------------------*/

Eigen::MatrixXd MoveArm::jtraj(Eigen::VectorXd q0, Eigen::VectorXd q1, int n_steps)
{ 
  int n_joints = q0.size();
  int time_scale = 1;
  Eigen::VectorXd time = Eigen::VectorXd::Zero(n_steps);

  for (int i=0; i < n_steps; i++)
  {
    time(i) = (double) i/(n_steps-1); // normalized time from 0 -> 1
  }

  // ROS_INFO_STREAM("Time vector:" << time);

  // Compute the polynomial coefficients
  int n_coeffs = 6;
  Eigen::MatrixXd poly_coeffs = Eigen::MatrixXd::Zero(n_coeffs,n_joints);
  for (int i = 0; i < n_joints; i++)
  {
    poly_coeffs(0,i) = 6*(q1(i) - q0(i));
    poly_coeffs(1,i) = -15*(q1(i) - q0(i));
    poly_coeffs(2,i) = 10*(q1(i) - q0(i));
    poly_coeffs(3,i) = 0; 
    poly_coeffs(4,i) = 0;
    poly_coeffs(5,i) = q0(i);
  }

  // ROS_INFO_STREAM("Poly coeffs matrix:" << poly_coeffs);
  // Compute the coordinates
  Eigen::MatrixXd time_matrix = Eigen::MatrixXd::Zero(n_steps,n_coeffs);
  for (int i = 0; i < n_steps; i++)
  {
    time_matrix(i,0) = pow(time(i),5);
    time_matrix(i,1) = pow(time(i),4);
    time_matrix(i,2) = pow(time(i),3);
    time_matrix(i,3) = pow(time(i),2); 
    time_matrix(i,4) = time(i);
    time_matrix(i,5) = 1;
  }
  
  // ROS_INFO_STREAM("Time matrix:" << time_matrix);
  Eigen::MatrixXd traj = Eigen::MatrixXd::Zero(n_steps,n_joints);

  traj = time_matrix * poly_coeffs;
  // ROS_INFO_STREAM("Traj matrix:" << traj);

  return traj;
}

/*--------------------------------------------------------------------
 * ------------ PRECOMPUTED CONFIGURATIONS SERVICES ------------------
 *------------------------------------------------------------------*/

bool MoveArm::HomeArm(move_excavator::HomeArm::Request  &req, move_excavator::HomeArm::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;

  ROS_INFO_STREAM("HOME ARM.");
  ROS_WARN_STREAM("Take me home, West Virginia.");
  // Message
  motion_control::ArmGroup q;
  q.q1 = 0;
  q.q2 = JOINT2_MIN;
  q.q3 = PI/2-JOINT2_MIN;
  q.q4 = -PI/2; // + PITCH
  // ROS_INFO_STREAM("Publishing joint angles (part 2):");
  // std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout).sleep();

  return true;
}

bool MoveArm::LowerArm(move_excavator::LowerArm::Request  &req, move_excavator::LowerArm::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;

  ROS_INFO_STREAM("DIG VOLATILE.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = 0;
    q.q2 = 0;
    q.q3 = PI/2;
    q.q4 = -PI/2; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(timeout/100).sleep();
  }
  return true;
}

bool MoveArm::Scoop(move_excavator::Scoop::Request  &req, move_excavator::Scoop::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;
  
  ROS_INFO_STREAM("SCOOP VOLATILE.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = 0;
    q.q2 = i*JOINT2_MAX/100;
    q.q3 = PI/2-i*JOINT2_MAX/100;
    q.q4 = -PI/2; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(timeout/100).sleep();
  }

  return true;
}

bool MoveArm::AfterScoop(move_excavator::AfterScoop::Request  &req, move_excavator::AfterScoop::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;

  ROS_INFO_STREAM("AFTER SCOOP.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = 0;
    q.q2 = i*JOINT2_MIN/100;
    q.q3 = PI/2-i*JOINT2_MIN/100;
    q.q4 = -PI/2; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(timeout/100).sleep();
  }

  return true;
}

bool MoveArm::ExtendArm(move_excavator::ExtendArm::Request  &req, move_excavator::ExtendArm::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;

  ROS_INFO_STREAM("EXTEND ARM.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = 0;
    q.q2 = JOINT2_MIN;
    q.q3 = JOINT3_MAX-i*(PI/2)/100;
    q.q4 = -PI/2+i*(PI/2)/100; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(timeout/100).sleep();
  }

 for (int i = 0; i<101; i++) 
  {
    q.q1 = i*heading_goal/100;
    q.q2 = JOINT2_MIN;
    q.q3 = JOINT3_MAX-(PI/2);
    q.q4 = -PI/2+(PI/2); // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(timeout/100).sleep();
  }

  return true;
}

bool MoveArm::DropVolatile(move_excavator::DropVolatile::Request  &req, move_excavator::DropVolatile::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;

  ROS_INFO_STREAM("DROP VOLATILE.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = heading_goal;
    q.q2 = JOINT2_MIN;
    q.q3 = JOINT3_MAX-PI/2;
    q.q4 = i*(PI/2)/100; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(timeout/100).sleep();
  }

  return true;
}

bool MoveArm::ExcavatorFK(move_excavator::ExcavatorFK::Request  &req, move_excavator::ExcavatorFK::Response &res)
{
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;

  geometry_msgs::PoseStamped pose;
  pose = solveFK(q1, q2, q3, q4);

  res.eePose = pose;
  return true;
}


bool MoveArm::GoToPose(move_excavator::GoToPose::Request  &req, move_excavator::GoToPose::Response &res)
{
  ROS_INFO_STREAM("MANIPULATION: Target point. Point:" << req.goal);

  camera_link_to_arm_mount = tf_buffer.lookupTransform(robot_name_+"_arm_mount", robot_name_+"_left_camera_optical", ros::Time(0), ros::Duration(1.0));
  tf2::doTransform(req.goal, goal_point_, camera_link_to_arm_mount);

  goal_point_.point.x -= 0.7;
  goal_point_.point.z -= 0.1;

  ROS_INFO_STREAM("MANIPULATION: Target new frame updated. Point:" << goal_point_);


  Eigen::VectorXd goal_xyzp = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd start_joints = Eigen::VectorXd::Zero(4);

  ros::spinOnce();
  start_joints << q1_curr_, q2_curr_, q3_curr_,q4_curr_;
  ROS_INFO_STREAM("Starting joint angles:" << start_joints);

  double timeout = req.timeLimit;
  goal_xyzp << req.goal.point.x, req.goal.point.y, req.goal.point.z, -PI/2;
  ROS_INFO_STREAM("Goal XYZP:" << goal_xyzp);

  Eigen::VectorXd goal_joints = solveIK(goal_xyzp);
  ROS_INFO_STREAM("Goal joint angles:" << goal_joints);

  int steps = 50;
  Eigen::MatrixXd trajectory = jtraj(start_joints, goal_joints, steps);
  ROS_INFO_STREAM("Trajectory:" << trajectory);

  motion_control::ArmGroup q;
  for (int i = 0; i<steps; i++) 
  {
    q.q1 = trajectory(i,0);
    q.q2 = trajectory(i,1);
    q.q3 = trajectory(i,2);
    q.q4 = trajectory(i,3);
    pubJointAngles.publish(q);
    // ROS_INFO_STREAM("Published joint angle cmds:" << q);
    ros::Duration(timeout/steps).sleep();
  }

  res.success = true;
  ROS_INFO_STREAM("Success:" <<  res.success);
  
  return true;
}

/*--------------------------------------------------------------------
 * ----------------- FORWARD KINEMATICS FUNCTIONS --------------------
 *------------------------------------------------------------------*/

geometry_msgs::PoseStamped MoveArm::solveFK(double q1, double q2, double q3, double q4)
{
  ROS_INFO_STREAM("FORWARD KINEMATICS.");

  theta_DH_ << 0, q1, q2, q3, q4;
  int N = theta_DH_.size();
  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  for (size_t i = 0; i < N; i++)
  {
      Ai = trotz(theta_DH_(i)) * transl(0, 0, d_DH_(i)) * transl(a_DH_(i), 0, 0) * trotx(alpha_DH_(i));
      if (i == 0)
      {
          T0Ti = Ai;
          ROS_INFO_STREAM("Ai:" << T0Ti);
      }
      else
      {
          T0Ti = T0Ti*Ai;
          ROS_INFO_STREAM("Final transformation" << T0Ti);
      }
  }
  T0Tn_ = T0Ti;

  Eigen::VectorXd pos_goal = Eigen::VectorXd::Zero(3);
  pos_goal = T0Tn_.block(0,3,3,1);


  Eigen::Matrix3d rot = T0Tn_.block(0,0,3,3);
  Eigen::Quaterniond q(rot);

  geometry_msgs::PoseStamped eePose;
  eePose.header.stamp = ros::Time::now();
  eePose.header.frame_id = robot_name_+"_base_link";
  eePose.pose.position.x = pos_goal[0];
  eePose.pose.position.y = pos_goal[1];
  eePose.pose.position.z = pos_goal[2];
  eePose.pose.orientation.x = q.x();
  eePose.pose.orientation.y = q.y();
  eePose.pose.orientation.z = q.z();
  eePose.pose.orientation.w = q.w();
  if (eePose.pose.orientation.w < 0) {
    eePose.pose.orientation.x *= -1;
    eePose.pose.orientation.y *= -1;
    eePose.pose.orientation.z *= -1;
    eePose.pose.orientation.w *= -1;
  }

  return eePose;
}

/*--------------------------------------------------------------------
 * ----------------- INVERSE KINEMATICS FUNCTIONS --------------------
 *------------------------------------------------------------------*/

Eigen::VectorXd MoveArm::solveIK(Eigen::VectorXd goal_xyzp)
{
  // End effector position
  double x_goal = goal_xyzp(0);
  double y_goal = goal_xyzp(1);
  double z_goal = goal_xyzp(2);
  double phi_goal = goal_xyzp(3);

  double r_goal = sqrt(x_goal*x_goal + y_goal*y_goal);
  double r_E = r_goal - a1_ - a4_*cos(phi_goal);
  double z_E = z_goal - d1_ - a4_*sin(phi_goal);
  double D = sqrt(z_E*z_E + r_E*r_E);

  double gamma = atan2(-z_E, -r_E) ;

  double q1_goal = atan2(y_goal,x_goal);
  double q2_goal = gamma + acos(-(r_E*r_E + z_E*z_E + a2_*a2_ - a3_*a3_)/(2*a2_*D));
  double q3_goal = atan2((z_E-a2_*sin(q2_goal))/a3_,(r_E-a2_*cos(q2_goal))/a3_);
  double q4_goal = phi_goal- (q2_goal + q3_goal);

  printf ("The goal joint angles are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  constrainAngle(q1_goal); constrainAngle(q2_goal); constrainAngle(q3_goal); constrainAngle(q4_goal);

  printf ("The constrained goal joint angles are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  limitJoint(q1_goal, JOINT1_MAX, JOINT1_MIN); limitJoint(q2_goal, JOINT2_MAX, JOINT2_MIN);
  limitJoint(q3_goal, JOINT3_MAX, JOINT3_MIN); limitJoint(q4_goal, JOINT4_MAX, JOINT4_MIN);

  printf ("The constrained goal joint angles with limits are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(4);

  q << q1_goal, q2_goal, q3_goal, q4_goal;

  return q;
}

/*--------------------------------------------------------------------
 * ---------------------- CONTROL FUNCTIONS --------------------------
 *------------------------------------------------------------------*/

Eigen::MatrixXd MoveArm::calculateJacobian()
{
  int N = theta_DH_.size();

  Eigen::Vector3d on(3);
  on = T0Tn_.block(0,3,3,1);

  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  Eigen::Vector3d oi(3);
  Eigen::Vector3d zi(3);

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,4);
  for (size_t i = 0; i < N; i++)
  {
    Ai = trotz(theta_DH_(i)) * transl(0, 0, d_DH_(i)) * transl(a_DH_(i), 0, 0) * trotx(alpha_DH_(i));
    if (i == 0)
    {
        T0Ti = Ai;
        zi = T0Ti.block(0,2,3,1);
        oi = T0Ti.block(0,3,3,1);
    }
    else
    {
        // Calculate Jacobian column with previous origin and z-axis (i-1)
        J.block(0, i-1, 3, 1) = zi.cross(on - oi);
        J.block(3, i-1, 3, 1) = zi;
        // Update origin and z-axis (i)
        T0Ti = T0Ti * Ai;
        zi = T0Ti.block(0,2,3,1);
        oi = T0Ti.block(0,3,3,1);
    }
  }
  return J;
}

Eigen::MatrixXd MoveArm::invertJacobian(Eigen::MatrixXd J)
{
  return J.completeOrthogonalDecomposition().pseudoInverse();
}


bool MoveArm::ControlInvJac(move_excavator::ControlInvJac::Request  &req, move_excavator::ControlInvJac::Response &res)
{
  
  Eigen::VectorXd goal_xyz = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd start_joints = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd goal_joints = Eigen::VectorXd::Zero(4);

  double timeout = req.timeLimit;
  goal_xyz << req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z;

  ros::spinOnce();
  
  geometry_msgs::PoseStamped ee_pose = solveFK(q1_curr_, q2_curr_, q3_curr_, q4_curr_);

  Eigen::VectorXd pos_current = T0Tn_.block(0,3,3,1);
  Eigen::VectorXd e = goal_xyz - pos_current;

  double pos_error = e.norm(); 

  ros::Rate control_rate(1/dt_);

  Eigen::VectorXd v = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,4);
  Eigen::MatrixXd pinvJ = Eigen::MatrixXd::Zero(4,6);

  motion_control::ArmGroup q;

  while (pos_error > max_pos_error_) 
  { 
    v = K_pos_ * e / e.norm();
    J = calculateJacobian();
    pinvJ = invertJacobian(J);

    qdot = pinvJ.block(0,0,4,3) * v;

    q.q1 = q1_curr_ + dt_*qdot(0);
    q.q2 = q2_curr_ + dt_*qdot(1);
    q.q3 = q3_curr_ + dt_*qdot(2);
    q.q4 = q4_curr_ + dt_*qdot(3);
    pubJointAngles.publish(q);
    
    control_rate.sleep();

    ros::spinOnce();
    ee_pose = solveFK(q1_curr_, q2_curr_, q3_curr_, q4_curr_);
    e = goal_xyz - pos_current;
    pos_error = e.norm();
  }

  return true;
}

/*!
 * \brief Creates and runs the MoveArm node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "move_arm");
  ros::NodeHandle nh("");

  ROS_INFO("Initializing Manipulation Controller.");
  MoveArm move_arm(nh);

  ros::Rate r(50);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;

} // end main()
