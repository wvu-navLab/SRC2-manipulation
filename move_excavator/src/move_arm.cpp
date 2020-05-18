/*!
 * \move_arm.cpp
 * \brief Move Arm Methods for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \date May 04, 2020
 */

#include "move_excavator/move_arm.h"

/*--------------------------------------------------------------------
 * MoveArm()
 * Constructor.
 *------------------------------------------------------------------*/


MoveArm::MoveArm(ros::NodeHandle & nh)
: nh_(nh)
{    
  // Node publishes individual joint positions
  pubJointAngles = nh_.advertise<motion_control::JointGroup>("/jointAngles", 1000);

  // Subscribers
  subJointsStates = nh_.subscribe("joint_states", 1000, &MoveArm::jointsCallback, this);

  // Service Servers
  serverHomeArm = nh_.advertiseService("home_arm", &MoveArm::HomeArm, this);
  serverExtendArm = nh_.advertiseService("extend_arm", &MoveArm::ExtendArm, this);
  serverDropVolatile = nh_.advertiseService("drop_volatile", &MoveArm::DropVolatile, this);
  serverDigVolatile = nh_.advertiseService("dig_volatile", &MoveArm::DigVolatile, this);
  serverScoop = nh_.advertiseService("scoop", &MoveArm::Scoop, this);

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
  th3 = atan2(h3,l3);
  th4 = atan2(h4,l4);

  th2_star = -th2;
  th3_star = th2-th3+PI/2;
  th4_star = th3-th4-PI/2;

  max_pos_error = 0.05;
  max_q4_error = 0.01;
  dt = 0.02;
  K_pos = 30;
  K_q4 = 0.2;

  // Denavit-Hartenberg Table
  a_DH << 0, 0, -a2, -a3, -a4;
  alpha_DH << 0, PI/2, 0, 0, 0;
  d_DH << h0, d1, 0, 0, 0;
  theta_DH << PI, q1_goal, q2_goal+th2_star, q3_goal+th3_star, q4_goal+th4_star;

  armControlEnabled = false;
} 

MoveArm::~MoveArm()
{
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
  double q1_ = req.heading;
  ROS_ERROR_STREAM("Take me home, West Virginia.");
  q.q1 = q1_;
  q.q2 = JOINT2_MIN;
  q.q3 = JOINT3_MAX;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles:");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  return true;
}

bool MoveArm::ExtendArm(move_excavator::ExtendArm::Request  &req, move_excavator::ExtendArm::Response &res)
{
  double q1_ = req.heading;
  ROS_ERROR_STREAM("Extending arm.");
  q.q1 = q1_;
  q.q2 = JOINT2_MIN*2/3;
  q.q3 = JOINT3_MIN;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles:");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  return true;
}

bool MoveArm::DropVolatile(move_excavator::DropVolatile::Request  &req, move_excavator::DropVolatile::Response &res)
{
  double q1_ = req.heading;
  ROS_ERROR_STREAM("Drop the mic.");
  q.q1 = q1_;
  q.q2 = JOINT2_MIN;
  q.q3 = JOINT3_MIN;
  q.q4 = JOINT4_MAX/2;
  ROS_ERROR_STREAM("Publishing joint angles:");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  return true;
}

bool MoveArm::DigVolatile(move_excavator::DigVolatile::Request  &req, move_excavator::DigVolatile::Response &res)
{
  double q1_ = req.heading;
  ROS_ERROR_STREAM("Drop the mic.");
  q.q1 = q1_;
  q.q2 = JOINT2_MAX/3;
  q.q3 = 0;
  q.q4 = JOINT4_MAX/2;
  ROS_ERROR_STREAM("Publishing joint angles:");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  return true;
}

bool MoveArm::Scoop(move_excavator::Scoop::Request  &req, move_excavator::Scoop::Response &res)
{
  double q1_ = req.heading;
  ROS_ERROR_STREAM("Drop the mic.");
  q.q1 = q1_;
  q.q2 = JOINT2_MAX/2;
  q.q3 = JOINT3_MAX/2;
  q.q4 = JOINT4_MAX;
  ROS_ERROR_STREAM("Publishing joint angles:");
  std::cout << q << std::endl;
  pubJointAngles.publish(q);
  return true;
}

/*--------------------------------------------------------------------
 * --------------------- KINEMATICS TOOLS ----------------------------
 *------------------------------------------------------------------*/

void MoveArm::forwardKinematics()
{
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
      }
  }
  T0Tn = T0Ti;
  
  pos_goal = T0Tn.block(0,3,3,1);
}

void MoveArm::inverseKinematics()
{
  double r = sqrt(x_goal*x_goal + y_goal*y_goal);
  double r_E = r - a4*cos(phi_goal);
  double z_E = z_goal - h0 - d1 - a4*sin(phi_goal);
  double D = sqrt(z_E*z_E + r_E*r_E);

  double gamma = atan2(-z_E/D, -r_E/D) ;

  q1_goal = atan2(y_goal,x_goal);
  q2_goal = gamma - acos(-(r_E*r_E + z_E*z_E + a2*a2 - a3*a3)/(2*a2*D));
  q3_goal = atan2((z_E-l2*sin(q2_goal))/a3,(r_E-a2*cos(q2_goal))/a3);
  q4_goal = phi_goal- (q2_goal + q3_goal);

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

void MoveArm::getJacobian()
{
  int N = theta_DH.size();

  Eigen::Vector3d on(3);
  on = T0Tn.block(0,3,3,1);

  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T0Ti(4,4);
  Eigen::Vector3d oi(3);
  Eigen::Vector3d zi(3);
  for (size_t i = 0; i < N; i++)
  {
    Ai = trotz(theta_DH(i)) * transl(0, 0, d_DH(i)) * transl(a_DH(i), 0, 0) * trotx(alpha_DH(i));
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
}

void MoveArm::invertJacobian(){
  pinvJ = J.completeOrthogonalDecomposition().pseudoInverse();
}

/*--------------------------------------------------------------------
 * -------------------------- CALLBACKS ------------------------------
 *------------------------------------------------------------------*/

void MoveArm::jointsCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (armControlEnabled)
  {
    // Find current angles and position
    double q1_current = msg->position[15];
    double q2_current = msg->position[0];
    double q3_current = msg->position[8];
    double q4_current = msg->position[7];

    printf ("Current joint angles.\n");
    theta_DH << PI, q1_current, q2_current, q3_current, q4_current;
    std::cout << theta_DH << std::endl;

    // Update current joint angles position
    theta_DH << PI, q1_current, q2_current+th2_star, q3_current+th3_star, q4_current+th4_star;

    forwardKinematics();
    Eigen::VectorXd pos_current = T0Tn.block(0,3,3,1);
    printf("Current Position Vector:\n");
    std::cout << pos_current << std::endl;
    Eigen::VectorXd e = pos_goal - pos_current;

    printf("Position Error Vector:\n");
    std::cout << e << std::endl;

    double pos_error = e.norm(); 

    if (pos_error > max_pos_error) 
    { 
      // Calculate end-effector velocity cmd
      Eigen::VectorXd v(3);
      Eigen::VectorXd qdot(4);
      v = K_pos * e / e.norm();
      // Find qdot using error and Jacobian
      getJacobian();
      printf("Jacobian:\n");
      std::cout << J << std::endl;
      invertJacobian();
      printf("Inverse Jacobian:\n");
      std::cout << pinvJ << std::endl;
      qdot = pinvJ.block(0,0,4,3) * v;
      printf("Commanded qdot:\n");
      std::cout << qdot << std::endl;
      printf ("Joint 1 ti (%2.6f).\n", q1_current);
      printf ("Joint 1 ti+1 (%2.6f).\n", q1_current + dt*qdot(0));
      // Publish new desired joint angles
      q.q1 = q1_current + dt*qdot(0);
      q.q2 = q2_current + dt*qdot(1);
      q.q3 = q3_current + dt*qdot(2);
      q.q4 = q4_current + dt*qdot(3);
      printf ("Published joint angles.\n");
      std::cout << q << std::endl;
      pubJointAngles.publish(q);
    }
  }
} // end publishCallback()


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