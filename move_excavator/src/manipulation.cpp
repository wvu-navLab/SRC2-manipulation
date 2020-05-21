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
  // Publishers
  pubFinished = nh_.advertise<move_excavator::ExcavationStatus>("/excavation_status", 1000);
  pubOdometryVolatile = nh_.advertise<nav_msgs::Odometry>("/odom_volatile", 1000);

  // Subscribers
  subJointStates = nh_.subscribe("joint_states", 1000, &Manipulation::jointStateCallback, this);
  subOdometry = nh_.subscribe("odom_truth", 1000, &Manipulation::odometryCallback, this);
  subBucketInfo = nh_.subscribe("bucket_info", 1000, &Manipulation::bucketCallback, this);
  subGoalVolatile = nh_.subscribe("/volatile_pos", 1000, &Manipulation::goalCallback, this);
  subMultiAgent = nh_.subscribe("/multiAgent", 1000, &Manipulation::alignCallback, this);
  subDebug =  nh_.subscribe("/debug", 1000, &Manipulation::debugCallback, this);

  // Service Clients
  clientFK = nh_.serviceClient<move_excavator::ExcavatorFK>("excavator_fk");

  clientHomeArm = nh_.serviceClient<move_excavator::HomeArm>("home_arm");
  clientDigVolatile = nh_.serviceClient<move_excavator::DigVolatile>("dig_volatile");
  clientScoop = nh_.serviceClient<move_excavator::Scoop>("scoop");
  clientExtendArm = nh_.serviceClient<move_excavator::ExtendArm>("extend_arm");
  clientDropVolatile = nh_.serviceClient<move_excavator::DropVolatile>("drop_volatile");
  
  clientRotateInPlace = nh_.serviceClient<driving_tools::RotateInPlace>("rotate_in_place");
  clientMoveForward = nh_.serviceClient<driving_tools::MoveForward>("move_forward");
  clientStop = nh_.serviceClient<driving_tools::Stop>("stop");
}

void Manipulation::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  q1_pos_ = msg->position[15];
  q2_pos_ = msg->position[0];
  q3_pos_ = msg->position[8];
  q4_pos_ = msg->position[7];
  ROS_INFO("Joint states updated.");
  ROS_INFO_STREAM("Values "<< q1_pos_<<" "<< q2_pos_<< " "<< q3_pos_<< " "<< q4_pos_);
}

void Manipulation::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Find current angles and position
  posx_ = msg->pose.pose.position.x;
  posy_ = msg->pose.pose.position.y;
  posz_ = msg->pose.pose.position.z;
  orientx_ = msg->pose.pose.orientation.x;
  orienty_ = msg->pose.pose.orientation.y;
  orientz_ = msg->pose.pose.orientation.z;
  orientw_ = msg->pose.pose.orientation.w;
  /* linvelx = msg->twist.linear.x;
  linvely = msg->twist.linear.y;
  linvelz = msg->twist.linear.z;
  angvelx = msg->twist.angular.x;
  angvely = msg->twist.angular.y;
  angvelz = msg->twist.angular.z;
  */
  ROS_INFO("Odometry updated.");
}

void Manipulation::bucketCallback(const srcp2_msgs::ExcavatorMsg::ConstPtr &msg)
{
  mass_in_bucket_ = msg->mass_in_bucket;
  if (mass_in_bucket_ != 0)
  {
    isBucketFull_ = true;
    ROS_INFO("A Pokémon is on the hook!");
  }
  else
  {
    isBucketFull_ = false;
    // ROS_INFO("Nothing on the bucket.");
  }
}

void Manipulation::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  x_goal_ = msg->pose.position.x;
  y_goal_ = msg->pose.position.y;
  z_goal_ = msg->pose.position.z;
  /* orientx_goal = msg->pose.orientation.x;
  orienty_goal = msg->pose.orientation.y;
  orientz_goal = msg->pose.orientation.z;
  orientw_goal = msg->pose.orientation.w;
 */
  pos_goal_ << x_goal_, y_goal_, z_goal_;

  ROS_INFO("Goal updated.");
}

void Manipulation::alignCallback(const move_excavator::MultiAgentState::ConstPtr &msg)
{
  isHaulerInView_ = msg->isRoverInView;
  isHaulerInRange_ = msg->isRoverInRange;
}

void Manipulation::debugCallback(const std_msgs::Int64::ConstPtr &msg)
{
  switch (msg->data)
  {
  case HOME_MODE:
    {
      executeHomeArm();
      ros::Duration(3).sleep();          
    }
    break;
  case DIG_MODE:
    {
      executeDig();
      ros::Duration(3).sleep();
    }
    break;
  case SCOOP_MODE:
    {
      getForwardKinematics();
      executeScoop();
      ros::Duration(3).sleep();
    }
    break;
  case EXTEND_MODE:
    {
      executeExtendArm();
      ros::Duration(3).sleep();
    }
    break;
  case DROP_MODE:
    {
      executeDrop();
      ros::Duration(3).sleep();
    }
    break;
  }
}


void Manipulation::getForwardKinematics()
{
  move_excavator::ExcavatorFK srv;
  motion_control::JointGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.joints = q;
  bool success = clientFK.call(srv);
  eePose = srv.response.eePose;

  ROS_INFO("Got the EE position using FK.");
  ROS_INFO_STREAM(eePose);


  double q2 = eePose.pose.orientation.x;
  double q3 = eePose.pose.orientation.y;
  double q4 = eePose.pose.orientation.z;
  double q1 = eePose.pose.orientation.w;

  double r, p, y;

  tf2::Quaternion quat(q2, q3, q4, q1); //or_x,or_y,or_z, and or_w are orientation message from nav_msg        
  tf2::Matrix3x3 m(quat); // q is your quaternion message, dont take just q, but normalize it with q.normalize()     
  m.getRPY(r, p, y); //get your roll pitch yaw from the quaternion message.   

  ROS_INFO_STREAM("Roll " << r << " Pitch " << p << " Yaw " << y);

  ros::Duration(10).sleep();
}

void Manipulation::updateLocalization()
{
  nav_msgs::Odometry msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = x_goal_ - eePose.pose.position.x;
  msg.pose.pose.position.y = y_goal_ - eePose.pose.position.y;
  msg.pose.pose.position.z = z_goal_ - eePose.pose.position.z;
  msg.pose.pose.orientation.x = 0.0; 
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.pose.orientation.w = 0.0;

  pubOdometryVolatile.publish(msg);
  ROS_INFO("An update in localization is available.");
}

void Manipulation::executeHomeArm()
{
  move_excavator::HomeArm srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientHomeArm.call(srv);
}

void Manipulation::executeDig()
{
  move_excavator::DigVolatile srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientDigVolatile.call(srv);
}

void Manipulation::executeScoop()
{
  move_excavator::Scoop srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientScoop.call(srv);
}

void Manipulation::executeExtendArm()
{
  move_excavator::ExtendArm srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientExtendArm.call(srv);
}

void Manipulation::executeDrop()
{
  move_excavator::DropVolatile srv;
  srv.request.heading = 0;
  srv.request.timeLimit = 100;
  bool success = clientDropVolatile.call(srv);
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
      if (manipulation.isManipulationStep_)
      {
        switch (manipulation.mode)
        {
        case HOME_MODE:
          {
            manipulation.executeHomeArm();
            ros::Duration(3).sleep();
            if(~manipulation.isBucketFull_)
            {
              manipulation.mode = DIG_MODE;
            }
            else
            {
              manipulation.mode = EXTEND_MODE;
            }              
          }
          break;
        case DIG_MODE:
          {
            manipulation.executeDig();
            ros::Duration(3).sleep();
            manipulation.mode = SCOOP_MODE;
          }
          break;
        case SCOOP_MODE:
          {
            manipulation.executeScoop();
            ros::Duration(3).sleep();
            manipulation.mode = HOME_MODE;
          }
          break;
        case EXTEND_MODE:
          {
            manipulation.executeExtendArm();
            ros::Duration(3).sleep();
            manipulation.mode = DROP_MODE;
          }
          break;
        case DROP_MODE:
          {
            manipulation.executeDrop();
            ros::Duration(3).sleep();
            manipulation.mode = HOME_MODE;
          }
          break;
        }
      }
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}