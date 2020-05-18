/*!
 * \manipulation.cpp
 * \brief Manipulation Planning for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@wvu.mix.edu
 * \date May 04, 2020
 */

#include "move_excavator/manipulation.h"

Manipulation::Manipulation(ros::NodeHandle & nh)
: nh_(nh)
{
  // Subscribers
  pubFinished = nh_.advertise<move_excavator::ExcavationStatus>("/excavation_status", 1000);

  // Subscribers
  subGoalVolatile = nh_.subscribe("/volatile_pos", 1000, &Manipulation::goalCallback, this);
  subBucketInfo = nh_.subscribe("bucket_info", 1000, &Manipulation::bucketCallback, this);
  subMultiAgent = nh_.subscribe("/multiAgent", 1000, &Manipulation::alignCallback, this);

  // Service Clients
  clientHomeArm = nh_.serviceClient<move_excavator::HomeArm>("home_arm");
  clientExtendArm = nh_.serviceClient<move_excavator::ExtendArm>("extend_arm");
  clientDropVolatile = nh_.serviceClient<move_excavator::DropVolatile>("drop_volatile");
  clientRotateInPlace = nh_.serviceClient<driving_tools::RotateInPlace>("rotate_in_place");
  clientMoveForward = nh_.serviceClient<driving_tools::MoveForward>("move_forward");
  clientStop = nh_.serviceClient<driving_tools::Stop>("stop");

  isBucketFull = false;
  isArmHome = false;
  isAligned = false;
  enableAlign = false;
  mass_collected = 0;
}

void Manipulation::goalCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  x_goal = msg->linear.x;
  y_goal = msg->linear.y;
  z_goal = msg->linear.z;
  phi_goal = msg->angular.z;

  pos_goal << x_goal, y_goal, z_goal;

  ROS_INFO("New position goal");

  if(mass_collected>98)
  {
    move_excavator::ExcavationStatus msg;
    msg.isFinished = true;
    msg.collectedMass = 100;
    pubFinished.publish(msg);
    mass_collected = 0;
  }
}

void Manipulation::bucketCallback(const srcp2_msgs::ExcavatorMsg::ConstPtr &msg)
{
  if(!isBucketFull)
  {
    double mass = msg->mass_in_bucket;
    mass_collected = mass_collected + mass;
    if (mass != 0)
    {
      isBucketFull = true;
      ROS_INFO("A Pokémon is on the hook!");
      ROS_INFO_STREAM("Mass extracted so far:" << mass_collected);
      ROS_INFO_STREAM("Mass extracted this time:" << mass);
      ROS_INFO("Homing the manipulator..");
      move_excavator::HomeArm srv;
      bool success = clientHomeArm.call(srv);
      ros::Duration(10).sleep();
      isArmHome = true;
    }
    else
    {
      ROS_INFO("Nothing on the bucket.");
      isBucketFull = false;
    }
  }
  else
  {
    if(isArmHome)
    {
      if(isAligned)
      {
        ROS_INFO("Extending Arm.");
        callExtendArm();
        ROS_INFO("Dropping Volatile.");
        callDropVolatile();
        enableAlign = false;
        isAligned = false;
      }
      else
      {
        ROS_INFO("Enabling Align.");
        enableAlign = true;
      }
    }
  }
}

void Manipulation::alignCallback(const move_excavator::MultiAgentState::ConstPtr &msg)
{
  bool isRoverInView = msg->isRoverInView;
  bool isRoverInRange = msg->isRoverInRange;
  if(enableAlign)
  {
    if(!isRoverInRange && !isRoverInView)
    {
      ROS_INFO("Rotating In Place.");
      driving_tools::RotateInPlace srv;
      srv.request.throttle = 0.5;
      bool success = clientRotateInPlace.call(srv);
    }
    else if(isRoverInView && !isRoverInRange)
    {
      ROS_INFO("Coming Closer.");
      driving_tools::MoveForward srv;
      srv.request.throttle = 0.5;
      bool success = clientMoveForward.call(srv);
    }
    else if(isRoverInView && isRoverInRange)
    {
      ROS_INFO("Stopping.");
      isAligned = true;
      enableAlign = false;
      driving_tools::Stop srv;
      srv.request.enableStop = true;
      bool success = clientStop.call(srv);
    }
    else
    {
      ROS_INFO("Something in front of me, but not the hauler.");
      driving_tools::Stop srv;
      srv.request.enableStop = true;
      bool success = clientStop.call(srv);
    }     
  }
}

void Manipulation::callExtendArm()
{
  isArmHome = false;
  move_excavator::ExtendArm srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientExtendArm.call(srv);
  ros::Duration(10).sleep();
}

void Manipulation::callDropVolatile()
{
  isArmHome = false;
  move_excavator::ExtendArm srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientExtendArm.call(srv);
  ros::Duration(10).sleep();
  isBucketFull = false;
}

/*!
 * \brief Creates and runs the Manipulation node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manipulation");
    ros::NodeHandle nh("");

    ROS_INFO("Initializing Manipulation Planner.");
    Manipulation manipulation(nh);

    ros::Rate rate(50);
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}