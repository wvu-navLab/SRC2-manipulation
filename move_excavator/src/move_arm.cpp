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
  pubJointAngles = nh_.advertise<motion_control::ArmGroup>("control/arm/joint_angles", 10);

  // Subscriber to joint states
  subJointStates = nh_.subscribe("joint_states", 10, &MoveArm::jointStateCallback, this);

  // Service Servers
  serverHomeArm = nh_.advertiseService("manipulation/home_arm", &MoveArm::HomeArm, this);
  serverExtendArm = nh_.advertiseService("manipulation/extend_arm", &MoveArm::ExtendArm, this);
  serverDropVolatile = nh_.advertiseService("manipulation/drop_volatile", &MoveArm::DropVolatile, this);
  serverLowerArm = nh_.advertiseService("manipulation/lower_arm", &MoveArm::LowerArm, this);
  serverScoop = nh_.advertiseService("manipulation/scoop", &MoveArm::Scoop, this);
  serverAfterScoop = nh_.advertiseService("manipulation/after_scoop", &MoveArm::AfterScoop, this);
  serverRetractArm = nh_.advertiseService("manipulation/retract_arm", &MoveArm::RetractArm, this);
  serverGoToPose = nh_.advertiseService("manipulation/go_to_pose", &MoveArm::GoToPose, this);
  serverControlInvJac = nh_.advertiseService("manipulation/control_inv_jac", &MoveArm::ControlInvJac, this);
  serverFK = nh_.advertiseService("manipulation/excavator_fk", &MoveArm::ExcavatorFK, this);

  // Link lengths
  a0_ = 0.70;
  d0_ = 0.10;

  a1_ = 0.19;
  d1_ = 0.12;
  alpha1_ = -PI/2;

  a2_ = 0.80;

  a3_ = 0.80;

  a4_ = 0.22;
  d4_ = 0.05;

  // Denavit-Hartenberg Table
  a_DH_ << a0_, a1_, a2_, a3_, a4_;
  alpha_DH_ << 0, alpha1_, 0, 0, 0;
  d_DH_ << d0_, d1_, 0, 0, d4_;
  theta_DH_ << 0, 0, 0, 0, 0;

  node_name_ ="move_arm";
  // Read params from yaml file
  if (ros::param::get("robot_name", robot_name_) == false)
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
 * ----------------------- UTIL FUNCTIONS ----------------------------
 *------------------------------------------------------------------*/

std::vector<double> MoveArm::LinearSpacedArray(double a, double b, std::size_t N)
{
    double h = (b - a) / static_cast<double>(N-1);
    std::vector<double> xs(N);
    std::vector<double>::iterator x;
    double val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
        *x = val;
    }
    return xs;
}

/*--------------------------------------------------------------------
 * -------------------- GEOMETRIC FUNCTIONS --------------------------
 *------------------------------------------------------------------*/

void MoveArm::ConstrainAngle(double& q)
{
    q = fmod(q + PI,2*PI);
    if (q < 0)
        q += 2*PI;
    q =  q - PI;
}

void MoveArm::LimitJoint(double& q, double max, double min)
{
  if (q > max)
  {
    q = max;
    ROS_WARN("Joint was limited to maximum value.");
  }
  if (q < min)
  {
    q = min;
    ROS_WARN("Joint was limited to minimum value.");
  }
}

Eigen::MatrixXd MoveArm::Trotx(double alpha)
{
  Eigen::MatrixXd T(4,4);
  T <<  1, 0,           0,            0,
        0, cos(alpha),  -sin(alpha),  0,
        0, sin(alpha),  cos(alpha),   0,
        0, 0,           0,            1;
  return T;
}

Eigen::MatrixXd MoveArm::Trotz(double theta)
{
  Eigen::MatrixXd T(4,4);
  T <<  cos(theta), -sin(theta),  0,  0,
        sin(theta), cos(theta),   0,  0,
        0,          0,            1,  0,
        0,          0,            0,  1;
  return T;
}

Eigen::MatrixXd MoveArm::Transl(double x, double y, double z)
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

Eigen::MatrixXd MoveArm::Jtraj(Eigen::VectorXd q0, Eigen::VectorXd q1, int n_steps)
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
  double duration = req.timeLimit;

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. HOMING ARM.");

  motion_control::ArmGroup q;
  q.q1 = heading_goal;
  q.q2 = JOINT2_MIN;
  q.q3 = PI/2-JOINT2_MIN;
  q.q4 = -PI/2; // + PITCH
  // ROS_INFO_STREAM("Publishing joint angles (part 2):");
  // std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(duration).sleep();

  return true;
}

bool MoveArm::LowerArm(move_excavator::LowerArm::Request  &req, move_excavator::LowerArm::Response &res)
{
  double heading_goal = req.heading;
  double duration = req.timeLimit;

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. LOWERING ARM.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = heading_goal;
    q.q2 = 0;
    q.q3 = PI/2;
    q.q4 = -PI/2; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(duration/100.0).sleep();
  }
  return true;
}

bool MoveArm::Scoop(move_excavator::Scoop::Request  &req, move_excavator::Scoop::Response &res)
{
  double heading_goal = req.heading;
  double duration = req.timeLimit;
  
  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. SCOOPING.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = heading_goal;
    q.q2 = i*JOINT2_MAX/100.0;
    q.q3 = PI/2-i*JOINT2_MAX/100.0;
    q.q4 = -PI/2; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(duration/100.0).sleep();
  }

  return true;
}

bool MoveArm::AfterScoop(move_excavator::AfterScoop::Request  &req, move_excavator::AfterScoop::Response &res)
{
  double heading_goal = req.heading;
  double duration = req.timeLimit;

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. AFTER SCOOP.");

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = q1_curr_;
    q.q2 = i*JOINT2_MIN/100.0;
    q.q3 = PI/2-i*JOINT2_MIN/100.0;
    q.q4 = -PI/2; // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(duration/100.0).sleep();
  }

  return true;
}

bool MoveArm::ExtendArm(move_excavator::ExtendArm::Request  &req, move_excavator::ExtendArm::Response &res)
{
  double range_goal = req.range;
  double heading_goal = req.heading;
  double duration = req.timeLimit;

  ROS_INFO_STREAM("EXTEND ARM.");

  double q2_goal;
  double q3_goal;

  if (range_goal < 0.42)
  {
    //RANGE TOO CLOSE
    res.success = false;
    res.approach = -1;
    q2_goal = JOINT2_MIN;
    q3_goal = JOINT3_MAX-(PI/2.0);
  }
  else if (range_goal >= 0.42 && range_goal < 1.45) 
  {
    res.success = true;
    res.approach = 0;
    q2_goal = JOINT2_MIN;
    q3_goal = JOINT3_MIN + (range_goal - 0.42)/(1.45-0.42)*(JOINT3_MAX-JOINT3_MIN);
  }
  else if (range_goal >= 1.45 && range_goal < 1.83)
  {
    res.success = true;
    res.approach = 0;
    q2_goal = -0.6544;
    q3_goal = -0.372 + (range_goal - 1.45)/(1.83-1.45)*(0.468-(-0.372));
  }
  else if (range_goal >= 1.83 && range_goal < 1.95)
  {
    res.success = true;
    res.approach = 0;
    q2_goal = -0.1785;
    q3_goal = -0.438 + (range_goal - 1.83)/(1.95-1.83)*(-0.1507-(-0.438));
  }
  else
  {
    //RANGE TOO FAR
    res.success = false;
    res.approach = 1;
    q2_goal = -0.1785;
    q3_goal = -0.1507;
  }

  motion_control::ArmGroup q;
  for (int i = 0; i<101; i++) 
  {
    q.q1 = q1_curr_ - (float) i/100.0*q1_curr_;
    q.q2 = q2_curr_ - (float) i/100.0*(q2_curr_-JOINT2_MIN);
    q.q3 = q3_curr_ - (float) i/100.0*(q3_curr_-JOINT3_MIN);
    q.q4 = 0 - (q.q2 + q.q3); // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(duration/(2*100.0)).sleep();
  }

  for (int i = 0; i<101; i++) 
  {
    q.q1 = (float) i/100.0*heading_goal;
    q.q2 = JOINT2_MIN - (float) i/100.0*(JOINT2_MIN-q2_goal);
    q.q3 = JOINT3_MIN - (float) i/100.0*(JOINT3_MIN-q3_goal);
    q.q4 = 0 - (q.q2 + q.q3); // + PITCH
    pubJointAngles.publish(q);
    ros::Duration(duration/(2*100.0)).sleep();
  }

  return true;
}

bool MoveArm::DropVolatile(move_excavator::DropVolatile::Request  &req, move_excavator::DropVolatile::Response &res)
{
  int type = req.type;
  double range = req.range;
  double duration = req.timeLimit;

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. DROP VOLATILE.");

  double q2_goal;
  double q3_goal;
  double wait_time;

  motion_control::ArmGroup q;
  if (type == 0)
  {
    for (int i = 0; i<101; i++)
    {
      q.q1 = q1_curr_;
      q.q2 = q2_curr_ + (float) i/100.0*(PI/28.0);
      q.q3 = q3_curr_;
      q.q4 = (float) i/100.0*(PI/1.2); // + PITCH
      pubJointAngles.publish(q);
      ros::Duration(duration/100.0).sleep();
    }
  }
  else
  {
    if(range < 1.10)
    {
      q2_goal = JOINT2_MIN;
      q3_goal = JOINT3_MAX *(3.0/4.0);
      wait_time = 4.0;
    }
    else if (range>= 1.10 && range <1.60)
    {
      q2_goal = q2_curr_ + JOINT2_MIN * (1.0/6.0);
      q3_goal = q3_curr_ + JOINT3_MAX;
      wait_time = 2.0;
    }
    else
    {
      q2_goal = JOINT2_MIN * (7.0/16.0);
      q3_goal = JOINT3_MAX *(3.0/4.0);
      wait_time = 2.0;
    }

    for (int i = 0; i<101; i++) 
    {
      q.q1 = q1_curr_;
      q.q2 = q2_curr_; // q.q2 = q2_curr_ - (float) i/100.0*(PI/24.0); // + (float) i/100.0*(PI/24.0);
      q.q3 = q3_curr_; // q.q3 = q3_curr_ + (float) i/100.0*(PI/4.0);
      q.q4 = q4_curr_ - (float) i/100.0*(q4_curr_-JOINT4_MAX); // + PITCH
      pubJointAngles.publish(q);
      ros::Duration(duration/(2*100.0)).sleep();
    }

    for (int i = 0; i<101; i++) 
    {
      q.q1 = q1_curr_;
      q.q2 = q2_curr_ - (float) i/100.0*(q2_curr_-q2_goal); // q.q2 = q2_curr_ - (float) i/100.0*(PI/24.0); // + (float) i/100.0*(PI/24.0);
      q.q3 = q3_curr_ - (float) i/100.0*(q3_curr_-q3_goal); // q.q3 = q3_curr_ + (float) i/100.0*(PI/4.0);
      q.q4 = JOINT4_MAX; // + PITCH
      pubJointAngles.publish(q);
      ros::Duration(duration/(2*100.0)).sleep();
    }

    ros::Duration(wait_time).sleep();
    
    for (int i = 0; i<101; i++) 
    {
      q.q1 = q1_curr_;
      q.q2 = q2_curr_;
      q.q3 = q3_curr_;
      q.q4 = q4_curr_;
      pubJointAngles.publish(q);
      ros::Duration(duration/(4*100.0)).sleep();
    }

  }
  return true;
}

bool MoveArm::RetractArm(move_excavator::RetractArm::Request  &req, move_excavator::RetractArm::Response &res)
{
  double heading_goal = req.heading;
  double duration = req.timeLimit;

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. RETRACTING ARM.");

  motion_control::ArmGroup q;
  q.q1 = -M_PI;
  q.q2 = JOINT2_MIN;
  q.q3 = JOINT3_MAX;
  q.q4 = 0 - q.q2 - q.q3; // + PITCH
  // q.q1 = -M_PI;
  // q.q2 = JOINT2_MAX;
  // q.q3 = JOINT3_MIN;
  // q.q4 = 0 - q.q2 - q.q3; // + PITCH
  // ROS_INFO_STREAM("Publishing joint angles (part 2):");
  // std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(duration).sleep();

  return true;
}

bool MoveArm::ExcavatorFK(move_excavator::ExcavatorFK::Request  &req, move_excavator::ExcavatorFK::Response &res)
{
  geometry_msgs::PoseStamped pose;
  if(req.use_current.data)
  {
    ros::spinOnce();
    pose = SolveFK(q1_curr_, q2_curr_, q3_curr_, q4_curr_);
  }
  else
  {
    pose = SolveFK(req.joints.q1, req.joints.q2, req.joints.q3, req.joints.q4);
  }

  res.eePose = pose;
  return true;
}


bool MoveArm::GoToPose(move_excavator::GoToPose::Request  &req, move_excavator::GoToPose::Response &res)
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Target point. Point:" << req.goal);
  // if (req.goal.header.frame_id != robot_name_+"_arm_mount")
  // {
  //   transform_to_arm_mount = tf_buffer.lookupTransform(robot_name_+"_arm_mount", req.goal.header.frame_id, ros::Time(0), ros::Duration(1.0));
  //   tf2::doTransform(req.goal, goal_point_, transform_to_arm_mount);
  // }

  goal_point_ = req.goal;

  // Correct displacement between the center of the rover and the base of the arm
  // For some reason, the arm_mount frame is at the center of the drone.
  goal_point_.point.x -= 0.7;
  goal_point_.point.z -= 0.1;

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Target updated (accounting for shift in x and z). Point:" << goal_point_);

  
  // Fake position for testing
  //goal_point_.point.x = 0.75;
  //goal_point_.point.y = 0.75;_arm_mount
  //goal_point_.point.z = 0.5;

  Eigen::VectorXd goal_xyzp = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd start_joints = Eigen::VectorXd::Zero(4);

  ros::spinOnce();
  start_joints << q1_curr_, q2_curr_, q3_curr_, q4_curr_;
  // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Starting joint angles:" << start_joints);

  double duration = req.timeLimit;
  goal_xyzp << goal_point_.point.x, goal_point_.point.y, goal_point_.point.z, 0; // The angle must be -15 to avoid droping the volatiles. Ideally it would be 0.
  // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Goal XYZP:" << goal_xyzp);

  std::pair<bool, Eigen::VectorXd> p = SolveIK(goal_xyzp);
  bool flag_found_solution = p.first;
  Eigen::VectorXd goal_joints = p.second;
  // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Goal joint angles:" << goal_joints);

  int steps = 50;
  Eigen::MatrixXd trajectory = Jtraj(start_joints, goal_joints, steps);
  // ROS_INFO_STREAM("Trajectory:" << trajectory);

  motion_control::ArmGroup q;
  for (int i = 0; i<steps; i++) 
  {
    q.q1 = trajectory(i,0);
    q.q2 = trajectory(i,1);
    q.q3 = trajectory(i,2);
    q.q4 = trajectory(i,3);
    pubJointAngles.publish(q);
    // ROS_INFO_STREAM("Published joint angle cmds:" << q);
    ros::Duration(duration/static_cast<double>(steps)).sleep();
    
  }

  res.success = true;
  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Success:" <<  res.success);
  
  return true;
}

/*--------------------------------------------------------------------
 * ----------------- FORWARD KINEMATICS FUNCTIONS --------------------
 *------------------------------------------------------------------*/

geometry_msgs::PoseStamped MoveArm::SolveFK(double q1, double q2, double q3, double q4)
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. FORWARD KINEMATICS.");

  // theta_DH_ << 0, q1, q2, q3, q4;
  int N = theta_DH_.size();
  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  for (size_t i = 0; i < N; i++)
  {
      Ai = Trotz(theta_DH_(i)) * Transl(0, 0, d_DH_(i)) * Transl(a_DH_(i), 0, 0) * Trotx(alpha_DH_(i));
      if (i == 0)
      {
          T0Ti = Ai;
          // ROS_INFO_STREAM("Ai:" << T0Ti);
      }
      else
      {
          T0Ti = T0Ti*Ai;
          // ROS_INFO_STREAM("Final transformation" << T0Ti);
      }
  }
  T0Tn_ = T0Ti;

  Eigen::VectorXd pos_goal = T0Tn_.block(0,3,3,1);
  Eigen::Matrix3d rot = T0Tn_.block(0,0,3,3);

  Eigen::Quaterniond q(rot);

  geometry_msgs::PoseStamped eePose;
  eePose.header.stamp = ros::Time::now();
  eePose.header.frame_id = robot_name_+"_base_footprint";
  eePose.pose.position.x = pos_goal(0);
  eePose.pose.position.y = pos_goal(1);
  eePose.pose.position.z = pos_goal(2);
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

std::pair<bool, Eigen::VectorXd> MoveArm::SolveIK(Eigen::VectorXd goal_xyzp)
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Starting Inverse Kinematics");
  // End effector position
  double x_goal = goal_xyzp(0);
  double y_goal = goal_xyzp(1);
  double z_goal = goal_xyzp(2);
  double phi_goal = goal_xyzp(3);

  double r_goal = sqrt(x_goal*x_goal + y_goal*y_goal);

  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Goal z: " << z_goal << ", goal r: " << r_goal << ".");

  // double r_E = r_goal - a1_ - a4_*cos(phi_goal);
  // double z_E = z_goal - d1_ - a4_*sin(phi_goal);
  // double D = sqrt(z_E*z_E + r_E*r_E);

  // double gamma = atan2(-z_E, -r_E) ;

  // double q2_goal = gamma + acos(-(r_E*r_E + z_E*z_E + a2_*a2_ - a3_*a3_)/(2*a2_*D));
  // double q3_goal = atan2((z_E-a2_*sin(q2_goal)),(r_E-a2_*cos(q2_goal)))-q2_goal;
  // double q4_goal = 0;
  double q1_goal, q2_goal, q3_goal, q4_goal;

  q1_goal = atan2(y_goal,x_goal);

  double r_test;
  double z_test;

  std::vector<double> q2_test = LinearSpacedArray(JOINT2_MIN, JOINT2_MAX, 50);
  std::vector<double> q3_test = LinearSpacedArray(JOINT3_MIN, JOINT3_MAX, 100);

  bool flag_found_solution = false;

  double error = 100;
  double min_q2_goal = JOINT2_MAX;
  double min_q3_goal = JOINT3_MAX;

  if (q1_goal > 1.75 && q1_goal < 2.60)
  {
    q2_goal = JOINT2_MIN;
    for (auto iter_q3 : q3_test)
    {
      r_test = RPolyFit(q2_goal, iter_q3);
      z_test = ZPolyFit(q2_goal, iter_q3);
      error = hypot(r_test - r_goal, z_test - z_goal);
      if (error < 0.1)
      {
        // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Found solution");
        flag_found_solution = true;
        if (iter_q3 < min_q3_goal)
        {
          min_q3_goal = iter_q3;
          q3_goal = iter_q3;
        }
      }
    }
  }
  else
  {
    for (auto iter_q2 : q2_test)
    {
      for (auto iter_q3 : q3_test)
      {
        r_test = RPolyFit(iter_q2, iter_q3);
        z_test = ZPolyFit(iter_q2, iter_q3);
        error = hypot(r_test - r_goal, z_test - z_goal);
        if (error < 0.3)
        {
          // ROS_INFO_STREAM("Found solution");
          // ROS_INFO_STREAM("Tested z: " << z_test << ", tested r: " << r_test << ".");
          // ROS_INFO_STREAM("Error " << error << ".");
          flag_found_solution = true;
          if (iter_q2 < min_q2_goal)
          {
            min_q2_goal = iter_q2;
            q2_goal = iter_q2;
            q3_goal = iter_q3;
          }
        }
      }
    }
  }

  q4_goal = phi_goal - (q2_goal + q3_goal);

  if (!flag_found_solution)
  {
    q1_goal = q1_curr_;
    q2_goal = q2_curr_;
    q3_goal = q3_curr_;
    q4_goal = q4_curr_;
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Found solution");
  }
  // ConstrainAngle(q1_goal);
  // ConstrainAngle(q2_goal); 
  // ConstrainAngle(q3_goal); 

  // ROS_INFO_STREAM("The constrained goal joint angles are q = (" << q1_goal << ", " << q2_goal << ", " << q3_goal << ", " << q4_goal << ").");

  // LimitJoint(q1_goal, JOINT1_MAX, JOINT1_MIN); 
  // LimitJoint(q2_goal, JOINT2_MAX, JOINT2_MIN);
  // LimitJoint(q3_goal, JOINT3_MAX, JOINT3_MIN);

  
  ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. The goal joint angles are q = (" << q1_goal << ", " << q2_goal << ", " << q3_goal << ", " << q4_goal << ").");

  // LimitJoint(q4_goal, JOINT4_MAX, JOINT4_MIN);

  // ROS_INFO_STREAM("The constrained goal joint angles with limits are q = (" << q1_goal << ", " << q2_goal << ", " << q3_goal << ", " << q4_goal << ").");

  Eigen::VectorXd q = Eigen::VectorXd::Zero(4);

  q << q1_goal, q2_goal, q3_goal, q4_goal;

  return std::make_pair(flag_found_solution, q);
}

inline double MoveArm::RPolyFit(double x, double y)
{
  return  r00 + r10*x + r01*y + r20*pow(x,2) + r11*x*y 
              + r02*pow(y,2) + r30*pow(x,3) + r21*pow(x,2)*y 
              + r12*x*pow(y,2) + r03*pow(y,3) + r40*pow(x,4) 
              + r31*pow(x,3)*y + r22*pow(x,2)*pow(y,2)
              + r13*x*pow(y,3) + r04*pow(y,4);
}

inline double MoveArm::ZPolyFit(double x, double y)
{
  return  z00 + z10*x + z01*y + z20*pow(x,2) + z11*x*y 
              + z02*pow(y,2) + z30*pow(x,3) + z21*pow(x,2)*y 
              + z12*x*pow(y,2) + z03*pow(y,3) + z40*pow(x,4) 
              + z31*pow(x,3)*y + z22*pow(x,2)*pow(y,2)
              + z13*x*pow(y,3) + z04*pow(y,4);
}

/*--------------------------------------------------------------------
 * ---------------------- CONTROL FUNCTIONS --------------------------
 *------------------------------------------------------------------*/

Eigen::MatrixXd MoveArm::CalculateJacobian()
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
    Ai = Trotz(theta_DH_(i)) * Transl(0, 0, d_DH_(i)) * Transl(a_DH_(i), 0, 0) * Trotx(alpha_DH_(i));
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

Eigen::MatrixXd MoveArm::InvertJacobian(Eigen::MatrixXd J)
{
  return J.completeOrthogonalDecomposition().pseudoInverse();
}


bool MoveArm::ControlInvJac(move_excavator::ControlInvJac::Request  &req, move_excavator::ControlInvJac::Response &res)
{
  ros::Time control_timer = ros::Time::now();

  Eigen::VectorXd goal_xyz = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd start_joints = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd goal_joints = Eigen::VectorXd::Zero(4);

  double duration = req.timeLimit;
  goal_xyz << req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z;

  ros::spinOnce();
  
  geometry_msgs::PoseStamped ee_pose = SolveFK(q1_curr_, q2_curr_, q3_curr_, q4_curr_);

  Eigen::VectorXd pos_current = T0Tn_.block(0,3,3,1);
  Eigen::VectorXd e = goal_xyz - pos_current;

  double pos_error = e.norm(); 

  ros::Rate control_rate(1);

  Eigen::VectorXd v = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,4);
  Eigen::MatrixXd pinvJ = Eigen::MatrixXd::Zero(4,6);

  motion_control::ArmGroup q;

  res.success = true;

  while (pos_error > max_pos_error_) 
  { 
    if (ros::Time::now() - control_timer > ros::Duration(duration))
    {
      break;
      res.success = false;
    }

    v = K_pos_ * e ;
    J = CalculateJacobian();
    pinvJ = InvertJacobian(J);

    qdot = pinvJ.block(0,0,3,3) * v;

    // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. The goal joint angles velocities are qdot = (" << qdot(0) << ", " << qdot(1) << ", " << qdot(2) << ").");

    q.q1 = q1_curr_ + dt_*qdot(0);
    q.q2 = q2_curr_ + dt_*qdot(1);
    q.q3 = q3_curr_ + dt_*qdot(2);
    LimitJoint(q.q2, JOINT2_MAX, JOINT2_MIN);
    LimitJoint(q.q3, JOINT3_MAX, JOINT3_MIN);
    q.q4 = 0 - (q.q2 + q.q3);
    
    // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Goal pose = (" << goal_xyz(0) << ", " << goal_xyz(1) << ", " << goal_xyz(2) << ").");
    // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. The goal joint angles are q = (" << q.q1 << ", " << q.q2 << ", " << q.q3 << ", " << q.q4 << ").");

    pubJointAngles.publish(q);
    
    control_rate.sleep();
    ros::spinOnce();

    ee_pose = SolveFK(q1_curr_, q2_curr_, q3_curr_, q4_curr_);

    pos_current = T0Tn_.block(0,3,3,1);

    // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. The current joint angles are q = (" << q1_curr_ << ", " << q2_curr_ << ", " << q3_curr_ << ", " << q4_curr_ << ").");
    // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Current pose = (" << pos_current(0) << ", " << pos_current(1) << ", " << pos_current(2) << ").");

    e = goal_xyz - pos_current;

    // ROS_INFO_STREAM("[" << robot_name_ << "] " << "MANIPULATION. Error = (" << e(0) << ", " << e(1) << ", " << e(2) << ").");

    pos_error = e.norm();

    // ROS_INFO_STREAM("Error norm = " << pos_error << ".");

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
