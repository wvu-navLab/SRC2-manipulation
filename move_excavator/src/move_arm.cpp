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
: nh_(nh)
{


  // Node publishes individual joint positions
  pubJointAngles = nh_.advertise<motion_control::ArmGroup>("control/arm/joint_angles", 1);

  // Service Servers
  serverHomeArm = nh_.advertiseService("manipulation/home_arm", &MoveArm::HomeArm, this);
  serverExtendArm = nh_.advertiseService("manipulation/extend_arm", &MoveArm::ExtendArm, this);
  serverDropVolatile = nh_.advertiseService("manipulation/drop_volatile", &MoveArm::DropVolatile, this);
  serverLowerArm = nh_.advertiseService("manipulation/lower_arm", &MoveArm::LowerArm, this);
  serverScoop = nh_.advertiseService("manipulation/scoop", &MoveArm::Scoop, this);
  serverAfterScoop = nh_.advertiseService("manipulation/after_scoop", &MoveArm::AfterScoop, this);
  serverFK = nh_.advertiseService("manipulation/excavator_fk", &MoveArm::ExcavatorFK, this);

  // Link lengths
  a0_ = 0.70;
  d0_ = 0.10;

  a1_ = 0.19;
  d1_ = 0.12;
  alpha1_ = -PI/2;

  a2_ = 0.80;

  a3_ = 0.80;

  a4_ = 0.33/2;

  // Denavit-Hartenberg Table
  a_DH << -a0_, -a1_, -a2_, -a3_, -a4_;
  alpha_DH << 0, , 0, 0, 0;
  d_DH << d0_, d1_, 0, 0, 0;
  theta_DH << 0, 0, 0, 0, 0;

  node_name = = 'move_arm';
  // Read params from yaml file
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

void Manipulation::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  int shoulder_yaw_joint_idx;
  int shoulder_pitch_joint_idx;
  int elbow_pitch_joint_idx;
  int wrist_pitch_joint_idx;
  int spawning_zone_joint_idx;

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

Eigen::MatrixXd MoveArm::jtraj(Eigen::VectorXd q0, Eigen::VectorXd q1, int steps)
{ 
  int n_joints = q0.size();
  int time_scale = 1;
  Eigen::VectorXd time = Eigen::VectorXd::Zero(steps);

  for (int i=0; i < steps; i++)
  {
    time(i) = i/(steps-1); // normalized time from 0 -> 1
  }

  // Compute the polynomial coefficients
  Eigen::MatrixXd poly_coeffs = Eigen::VectorXd::Zero(n_joints,steps);
  for (int i = 0; i < n_joints; i++)
  {
    poly_coeffs(i,0) = 6*(q1(i) - q0(i));
    poly_coeffs(i,1) = -15*(q1(i) - q0(i));
    poly_coeffs(i,2) = 10*(q1(i) - q0(i));
    poly_coeffs(i,3) = 0; 
    poly_coeffs(i,4) = 0;
    poly_coeffs(i,5) = q0(i);
  }

  // Compute the coordinates
  Eigen::MatrixXd time_matrix = Eigen::VectorXd::Zero(n_joints,steps);
  for (int i = 0; i < n_joints; i++)
  {
    time_matrix(i,0) = pow(time(i),5);
    time_matrix(i,1) = pow(time(i),4);
    time_matrix(i,2) = pow(time(i),3);
    time_matrix(i,3) = pow(time(i),2); 
    time_matrix(i,4) = time(i);
    time_matrix(i,5) = 1;
  }
  
  Eigen::MatrixXd traj = Eigen::VectorXd::Zero(steps,n_joints);

  traj = time_matrix * poly_coeffs.transpose();

  return traj;
}

/*--------------------------------------------------------------------
 * ------------ PRECOMPUTED CONFIGURATIONS SERVICES ------------------
 *------------------------------------------------------------------*/

bool MoveArm::HomeArm(move_excavator::HomeArm::Request  &req, move_excavator::HomeArm::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;

  ROS_INFO_STREAM("HOME ARM.");
  ROS_WARN_STREAM("Take me home, West Virginia.");
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
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;

  ROS_INFO_STREAM("DIG VOLATILE.");

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
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;
  
  ROS_INFO_STREAM("SCOOP VOLATILE.");

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
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;
  
  ROS_INFO_STREAM("AFTER SCOOP.");

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
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;

  ROS_INFO_STREAM("EXTEND ARM.");

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
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;

  ROS_INFO_STREAM("DROP VOLATILE.");

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

  geometry_msgs::Pose pose;
  pose = calculateFK(q1, q2, q3, q4);

  res.eePose = pose;
  return true;
}


/*--------------------------------------------------------------------
 * ----------------- FORWARD KINEMATICS FUNCTIONS --------------------
 *------------------------------------------------------------------*/

geometry_msgs::Pose MoveArm::calculateFK(double q1, double q2, double q3, double q4)
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
      }
      else
      {
          T0Ti = T0Ti*Ai;
          ROS_INFO_STREAM("Transformation"<<T0Ti);
      }
  }
  T0Tn_ = T0Ti;
  pos_goal = T0Tn_.block(0,3,3,1);


  Eigen::Matrix3d rot = T0Tn_.block(0,0,3,3);
  Eigen::Quaterniond q(rot);

  geometry_msgs::Pose eePose;

  eePose.position.x = pos_goal[0];
  eePose.position.y = pos_goal[1];
  eePose.position.z = pos_goal[2];
  eePose.orientation.x = q.x();
  eePose.orientation.y = q.y();
  eePose.orientation.z = q.z();
  eePose.orientation.w = q.w();
  if (eePose.orientation.w < 0) {
    eePose.orientation.x *= -1;
    eePose.orientation.y *= -1;
    eePose.orientation.z *= -1;
    eePose.orientation.w *= -1;
  }

  return eePose;
}

/*--------------------------------------------------------------------
 * ----------------- INVERSE KINEMATICS FUNCTIONS --------------------
 *------------------------------------------------------------------*/

Eigen::VectorXd MoveArm::calculateIK(Eigen::VectorXd goal)
{
  // End effector position
  double x_goal = goal(0);
  double y_goal = goal(1);
  double z_goal = goal(2);
  double phi_goal = -PI/2; // goal(0);

  double r = sqrt(x_goal*x_goal + y_goal*y_goal);
  double r_E = r - a4_*cos(phi_goal);
  double z_E = z_goal - d0_ - d1_ - a4_*sin(phi_goal);
  double D = sqrt(z_E*z_E + r_E*r_E);

  double gamma = atan2(-z_E/D, -r_E/D) ;

  double q1_goal = atan2(y_goal,x_goal);
  double q2_goal = gamma - acos(-(r_E*r_E + z_E*z_E + a2_*a2_ - a3_*a3_)/(2*a2_*D));
  double q3_goal = atan2((z_E-l2*sin(q2_goal))/a3_,(r_E-a2_*cos(q2_goal))/a3_);
  double q4_goal = phi_goal- (q2_goal + q3_goal);

  printf ("The goal joint angles are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  constrainAngle(q1_goal);
  constrainAngle(q2_goal);
  constrainAngle(q3_goal);
  constrainAngle(q4_goal);

  printf ("The constrained goal joint angles are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);

  limitJoint(q1_goal, JOINT1_MAX, JOINT1_MIN);
  limitJoint(q2_goal, JOINT2_MAX, JOINT2_MIN);
  limitJoint(q3_goal, JOINT3_MAX, JOINT3_MIN);
  limitJoint(q4_goal, JOINT4_MAX, JOINT4_MIN);

  printf ("The constrained goal joint angles with limits are q = (%2.2f, %2.2f, %2.2f, %2.2f).\n", q1_goal, q2_goal, q3_goal, q4_goal);
}

/*--------------------------------------------------------------------
 * ---------------------- CONTROL FUNCTIONS --------------------------
 *------------------------------------------------------------------*/

Eigen::MatrixXd MoveArm::getJacobian(Eigen::MatrixXd T0Tn)
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
  double x = req.joints.q1;
  double y = req.joints.q2;
  double z = req.joints.q3;

  Eigen::VectorXd pos_goal_ << x, y, z;

  ros.spinOnce();
  // Find current angles and position
  theta_DH_ << PI, q1_curr_, q2_curr_, q3_curr_, q4_curr_;
  pose = calculateFK(q1_curr_, q2_curr_, q3_curr_, q4_curr_);

  Eigen::VectorXd pos_current = T0Tn_.block(0,3,3,1);
  Eigen::VectorXd e = pos_goal_ - pos_current;

  double pos_error = e.norm(); 

  if (pos_error > max_pos_error_) 
  { 
    Eigen::VectorXd v(3);
    Eigen::VectorXd qdot(4);
    v = K_pos_ * e / e.norm();

    Eigen::MatrixXd J = getJacobian();
    Eigen::MatrixXd pinvJ = invertJacobian(J);

    qdot = pinvJ.block(0,0,4,3) * v;

    q.q1 = q1_current + dt*qdot(0);
    q.q2 = q2_current + dt*qdot(1);
    q.q3 = q3_current + dt*qdot(2);
    q.q4 = q4_current + dt*qdot(3);
    pubJointAngles.publish(q);
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
