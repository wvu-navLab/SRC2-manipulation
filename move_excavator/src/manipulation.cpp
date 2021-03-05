/*!
 * \manipulation.cpp
 * \brief Manipulation Planning for SRC2 Rovers
 *
 * Manipulation creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date May 04, 2020
 */

#include "move_excavator/manipulation.h"

Manipulation::Manipulation(ros::NodeHandle & nh)
: nh_(nh)
{
  // Publishers
  pubExcavationStatus = nh_.advertise<move_excavator::ExcavationStatus>("manipulation/feedback", 10);

  // Subscribers
  // subOdometry = nh_.subscribe("localization/odometry/sensor_fusion", 10, &Manipulation::odometryCallback, this);
  // subHaulerOdom =  nh_.subscribe("/small_hauler_1/localization/odometry/sensor_fusion", 10, &Manipulation::haulerOdomCallback, this);
  subJointStates = nh_.subscribe("joint_states", 1, &Manipulation::jointStateCallback, this);
  subBucketInfo = nh_.subscribe("scoop_info", 1, &Manipulation::bucketCallback, this);
  subGoalVolatile = nh_.subscribe("manipulation/volatile_pose", 1, &Manipulation::goalCallback, this);
  subManipulationState =  nh_.subscribe("manipulation/state", 1, &Manipulation::manipulationStateCallback, this);
  // subLaserScanHauler = nh_.subscribe("/small_hauler_1/laser/scan", 1, &Manipulation::laserCallbackHauler, this);

  // Service Clients
  clientFK = nh_.serviceClient<move_excavator::ExcavatorFK>("manipulation/excavator_fk");
  clientHomeArm = nh_.serviceClient<move_excavator::HomeArm>("manipulation/home_arm");
  clientLowerArm = nh_.serviceClient<move_excavator::LowerArm>("manipulation/lower_arm");
  clientScoop = nh_.serviceClient<move_excavator::Scoop>("manipulation/scoop");
  clientAfterScoop = nh_.serviceClient<move_excavator::AfterScoop>("manipulation/after_scoop");
  clientExtendArm = nh_.serviceClient<move_excavator::ExtendArm>("manipulation/extend_arm");
  clientDropVolatile = nh_.serviceClient<move_excavator::DropVolatile>("manipulation/drop_volatile");
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

  q1_pos_ = msg->position[shoulder_yaw_joint_idx];  // TODO: Get ids from the message names
  q2_pos_ = msg->position[shoulder_pitch_joint_idx];
  q3_pos_ = msg->position[elbow_pitch_joint_idx];
  q4_pos_ = msg->position[wrist_pitch_joint_idx];
}

void Manipulation::bucketCallback(const srcp2_msgs::ExcavatorScoopMsg::ConstPtr &msg)
{
  isBucketFull_ = msg->volatile_clod_mass || msg->regolith_clod_mass;
  found_volatile_  = msg->volatile_clod_mass;
}

void Manipulation::goalCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  x_goal_ = msg->position.x;
  y_goal_ = msg->position.y;
  z_goal_ = msg->position.z;
  orientx_goal_ = msg->orientation.x;
  orienty_goal_ = msg->orientation.y;
  orientz_goal_ = msg->orientation.z;
  orientw_goal_ = msg->orientation.w;

  pos_goal_ << x_goal_, y_goal_, z_goal_;

  ROS_INFO_STREAM("MANIPULATION: Goal volatile updated. Pose:" << *msg);
}

void Manipulation::manipulationStateCallback(const std_msgs::Int64::ConstPtr &msg)
{
  ROS_INFO_STREAM("MANIPULATION: New manipulation state commanded:" << msg->data);
  switch (msg->data)
  {
  case STOP:
    {
      executeHomeArm(2);
      manipulation_enabled_ = false;
    }
    break;
  case HOME_MODE:
    {
      executeHomeArm(2);
    }
    break;
  case LOWER_MODE:
    {
      executeLowerArm(2);
    }
    break;
  case SCOOP_MODE:
    {
      executeScoop(2);
      executeAfterScoop(2);
    }
    break;
  case EXTEND_MODE:
    {
      executeExtendArm(4);
    }
    break;
  case DROP_MODE:
    {
      executeDrop(2);
    }
    break;
  case START:
    {
      manipulation_enabled_ = true;
      found_volatile_ = false;
      manipulation_start_time_ = ros::Time::now();
      ROS_WARN("MANIPULATION: has started!");
    }
    break;
  }
}

void Manipulation::getRelativePosition()
{
  relative_heading_ = 1.2;
  // ROS_INFO_STREAM("Hauler odometry updated. Pose:" << msg->pose.pose);
  // ROS_INFO_STREAM("Range:" << relative_range);
  // ROS_INFO_STREAM("Is Hauler in range:" << hauler_in_range_);
  // ROS_INFO_STREAM("Relative heading:" << yaw_);
}

void Manipulation::executeHomeArm(double timeout)
{
  move_excavator::HomeArm srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = 0;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientHomeArm.call(srv);
}

void Manipulation::executeLowerArm(double timeout)
{
  move_excavator::LowerArm srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = volatile_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientLowerArm.call(srv);
}

void Manipulation::executeScoop(double timeout)
{
  move_excavator::Scoop srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = volatile_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientScoop.call(srv);
}

void Manipulation::executeAfterScoop(double timeout)
{
  move_excavator::AfterScoop srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = volatile_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientAfterScoop.call(srv);
}


void Manipulation::executeExtendArm(double timeout)
{
  move_excavator::ExtendArm srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = relative_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientExtendArm.call(srv);
}

void Manipulation::executeDrop(double timeout)
{
  move_excavator::DropVolatile srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = relative_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientDropVolatile.call(srv);
}


void Manipulation::getForwardKinematics(double timeout)
{
  move_excavator::ExcavatorFK srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.joints = q;

  bool success = clientFK.call(srv);
  eePose_ = srv.response.eePose;
}

void Manipulation::outputManipulationStatus()
{
  move_excavator::ExcavationStatus msg;
  msg.isFinished = true;
  msg.foundVolatile = found_volatile_;
  msg.collectedMass = 0;
  msg.haulerInRange = hauler_in_range_;
  pubExcavationStatus.publish(msg);
  ROS_INFO("MANIPULATION: has finished. Publishing to State Machine.");
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

    ROS_INFO("Initializing Manipulation State Machine Node.");
    Manipulation manipulation(nh);

    ros::Rate rate(50);
    while(ros::ok())
    {
      if (manipulation.manipulation_enabled_ && (ros::Time::now() - manipulation.manipulation_start_time_) < ros::Duration(420))
      {
        switch (manipulation.mode)
        {
        case HOME_MODE:
          {
            manipulation.executeHomeArm(2);
            if(manipulation.isBucketFull_)
            {
              manipulation.mode = EXTEND_MODE;
              // ROS_INFO_STREAM("Is bucket full: " << manipulation.isBucketFull_);
            }
            else
            {
              manipulation.mode = LOWER_MODE;
              // ROS_INFO_STREAM("Is bucket full: " << manipulation.isBucketFull_);
            }
          }
          break;
        case LOWER_MODE:
          {
            manipulation.executeLowerArm(2);
            manipulation.mode = SCOOP_MODE;
          }
          break;
        case SCOOP_MODE:
          {
            manipulation.executeScoop(2);
            manipulation.executeAfterScoop(2);
            manipulation.mode = HOME_MODE;
          }
          break;
        case EXTEND_MODE:
          {
            manipulation.executeExtendArm(4);
            ros::Duration(5).sleep();
            manipulation.mode = DROP_MODE;
          }
          break;
        case DROP_MODE:
          {
            manipulation.executeDrop(3);
            manipulation.manipulation_enabled_ = false;
            manipulation.mode = HOME_MODE;
          }
          break;
        }
      }
      else if (manipulation.manipulation_enabled_ && (ros::Time::now() - manipulation.manipulation_start_time_) > ros::Duration(420))
      {
        manipulation.manipulation_enabled_ = false;
        manipulation.outputManipulationStatus();
        manipulation.found_volatile_ = false;
        ROS_ERROR_STREAM("MANIPULATION: interrupted by timeout.");
      }
      else
      {
        manipulation.manipulation_enabled_ = false;
        manipulation.found_volatile_ = false;
        ROS_ERROR_THROTTLE(30,"MANIPULATION: disabled.");
      }
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
