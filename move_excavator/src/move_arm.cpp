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
  pubJointAngles = nh_.advertise<motion_control::JointGroup>("manipulation/arm_joint_angles", 1000);

  // Service Servers
  serverHomeArm = nh_.advertiseService("manipulation/home_arm", &MoveArm::HomeArm, this);
  serverExtendArm = nh_.advertiseService("manipulation/extend_arm", &MoveArm::ExtendArm, this);
  serverDropVolatile = nh_.advertiseService("manipulation/drop_volatile", &MoveArm::DropVolatile, this);
  serverDigVolatile = nh_.advertiseService("manipulation/dig_volatile", &MoveArm::DigVolatile, this);
  serverScoop = nh_.advertiseService("manipulation/scoop", &MoveArm::Scoop, this);
  serverAfterScoop = nh_.advertiseService("manipulation/after_scoop", &MoveArm::AfterScoop, this);
  serverFK = nh_.advertiseService("manipulation/excavator_fk", &MoveArm::ExcavatorFK, this);

  // Link lengths
  h0 = 0.3556;

  l1 = 0.1128;
  h1 = 0.0100;
  d1 = sqrt(l1*l1 + h1*h1);

  l2 = 1.5644;
  h2 = 0.9644;
  a2 = sqrt(l2*l2 + h2*h2);

  l3 = 0.7394;
  h3 = 0.5356;
  a3 = sqrt(l3*l3 + h3*h3);

  l4 = 0.2500;
  h4 = 0.2855;
  a4 = sqrt(l4*l4 + h4*h4);

  th2 = atan2(h2,l2);
  th3 = atan2(l3,h3);
  th4 = atan2(h4,l4);

  th2_star = -th2;
  th3_star = th2-th3+PI/2;
  th4_star = th3-th4-PI/2;

  // Denavit-Hartenberg Table
  a_DH << 0, 0, -a2, -a3, -a4;
  alpha_DH << 0, PI/2, 0, 0, 0;
  d_DH << h0, d1, 0, 0, 0;
  theta_DH << PI, q1_goal, q2_goal+th2_star, q3_goal+th3_star, q4_goal+th4_star;
}

/*--------------------------------------------------------------------
 * -------------------- GEOMETRY CALC TOOLS --------------------------
 *------------------------------------------------------------------*/

void MoveArm::constrainAngle(double& q_)
{
    q_ = fmod(q_ + PI,2*PI);
    if (q_ < 0)
        q_ += 2*PI;
    q_ =  q_ - PI;
}

void MoveArm::limitJoint(double& q_, double max_, double min_)
{
  (q_ > max_)? max_: q_;
  (q_ < min_)? min_: q_;
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

  ROS_ERROR_STREAM("Take me home, West Virginia.");
  q.q1 = q1;
  q.q2 = JOINT2_MIN;
  q.q3 = JOINT3_MAX/2;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles (part 1):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout/2).sleep();

  q.q1 = heading_goal;
  q.q2 = JOINT2_MIN;
  q.q3 = JOINT3_MAX;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles (part 2):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout/2).sleep();

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

  ROS_ERROR_STREAM("Extending arm.");
  q.q1 = q1;
  q.q2 = JOINT2_MIN*2/3;
  q.q3 = 0; //JOINT3_MIN;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles (part 1):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout/2).sleep();

  q.q1 = heading_goal;
  q.q2 = JOINT2_MIN*2/3;
  q.q3 = 0; //JOINT3_MIN;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles (part 2):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout/2).sleep();

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

  ROS_ERROR_STREAM("Hopefully I'm dropping the load in the hauler.");
  q.q1 = heading_goal;
  q.q2 = JOINT2_MIN*1/3;
  q.q3 = JOINT3_MIN;
  q.q4 = JOINT4_MAX/2;
  ROS_ERROR_STREAM("Publishing joint angles (part 1):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout*3/4).sleep();

  q.q1 = heading_goal;
  q.q2 = JOINT2_MIN*3/4;
  q.q3 = JOINT3_MIN;
  q.q4 = JOINT4_MAX/2;
  ROS_ERROR_STREAM("Publishing joint angles (part 2):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout*1/4).sleep();

  return true;
}

bool MoveArm::DigVolatile(move_excavator::DigVolatile::Request  &req, move_excavator::DigVolatile::Response &res)
{
  double heading_goal = req.heading;
  double timeout = req.timeLimit;
  double q1 = req.joints.q1;
  double q2 = req.joints.q2;
  double q3 = req.joints.q3;
  double q4 = req.joints.q4;

  ROS_ERROR_STREAM("Drop the mic.");
  q.q1 = heading_goal;
  q.q2 = JOINT2_MAX*1/3;
  q.q3 = 0;
  q.q4 = JOINT4_MAX/4;
  ROS_ERROR_STREAM("Publishing joint angles (part 1):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout/2).sleep();

  q.q1 = heading_goal;
  q.q2 = JOINT2_MAX*2/3;
  q.q3 = JOINT3_MAX/3;
  q.q4 = JOINT4_MAX/2;
  ROS_ERROR_STREAM("Publishing joint angles (part 2):");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout/2).sleep();

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
  
  ROS_ERROR_STREAM("One scoop of volatile for us.");
  q.q1 = heading_goal;
  q.q2 = JOINT2_MAX*2/3;
  q.q3 = JOINT3_MAX/2;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles :");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);

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
  
  ROS_ERROR_STREAM("After scoop.");
  q.q1 = heading_goal;
  q.q2 = JOINT2_MAX*1/3;
  q.q3 = JOINT3_MAX/2;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles :");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  ros::Duration(timeout).sleep();

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

geometry_msgs::Pose MoveArm::calculateFK(double q1, double q2, double q3, double q4)
{
  theta_DH << PI, q1, q2 + th2_star, q3 + th3_star, q4 + th4_star;
  int N = theta_DH.size();
  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  for (size_t i = 0; i < N; i++)
  {
      Ai = trotz(theta_DH(i)) * transl(0, 0, d_DH(i)) * transl(a_DH(i), 0, 0) * trotx(alpha_DH(i));
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
  T0Tn = T0Ti;
  pos_goal = T0Tn.block(0,3,3,1);


  Eigen::Matrix3d rot = T0Tn.block(0,0,3,3);
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
