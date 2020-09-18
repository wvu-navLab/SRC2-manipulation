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
  pubMeasurementUpdate = nh_.advertise<geometry_msgs::Pose>("position_update", 1);

  // Subscribers
  subOdometry = nh_.subscribe("localization/odometry/sensor_fusion", 10, &Manipulation::odometryCallback, this);
  subHaulerOdom =  nh_.subscribe("/hauler_1/localization/odometry/sensor_fusion", 10, &Manipulation::haulerOdomCallback, this);
  subJointStates = nh_.subscribe("joint_states", 1, &Manipulation::jointStateCallback, this);
  subBucketInfo = nh_.subscribe("bucket_info", 1, &Manipulation::bucketCallback, this);
  subGoalVolatile = nh_.subscribe("manipulation/volatile_pose", 1, &Manipulation::goalCallback, this);
  subManipulationState =  nh_.subscribe("manipulation/state", 1, &Manipulation::manipulationStateCallback, this);
  subLaserScanHauler = nh_.subscribe("/hauler_1/laser/scan", 1, &Manipulation::laserCallbackHauler, this);

  // Service Clients
  clientFK = nh_.serviceClient<move_excavator::ExcavatorFK>("manipulation/excavator_fk");

  clientHomeArm = nh_.serviceClient<move_excavator::HomeArm>("manipulation/home_arm");
  clientDigVolatile = nh_.serviceClient<move_excavator::DigVolatile>("manipulation/dig_volatile");
  clientScoop = nh_.serviceClient<move_excavator::Scoop>("manipulation/scoop");
  clientAfterScoop = nh_.serviceClient<move_excavator::AfterScoop>("manipulation/after_scoop");
  clientExtendArm = nh_.serviceClient<move_excavator::ExtendArm>("manipulation/extend_arm");
  clientDropVolatile = nh_.serviceClient<move_excavator::DropVolatile>("manipulation/drop_volatile");
}

void Manipulation::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  q1_pos_ = msg->position[15];  // TODO: Get ids from the message names
  q2_pos_ = msg->position[0];
  q3_pos_ = msg->position[8];
  q4_pos_ = msg->position[7];
  // ROS_INFO("Joint states updated.");
  // ROS_INFO_STREAM("Values "<< q1_pos_<<" "<< q2_pos_<< " "<< q3_pos_<< " "<< q4_pos_);
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

  tf2::Quaternion quat(orientx_, orienty_, orientz_, orientw_); //or_x,or_y,or_z, and or_w are orientation message from nav_msg
  tf2::Matrix3x3 rover_rot_matrix_(quat); // q is your quaternion message, dont take just q, but normalize it with q.normalize()
  rover_rot_matrix_.getRPY(roll_, pitch_, yaw_); //get your roll pitch yaw from the quaternion message.

  // ROS_INFO_STREAM("Excavator odometry updated. Pose:" << msg->pose.pose);
}

void Manipulation::haulerOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Find current angles and position
  posx_hauler_ = msg->pose.pose.position.x;
  posy_hauler_ = msg->pose.pose.position.y;
  posz_hauler_ = msg->pose.pose.position.z;
  orientx_hauler_ = msg->pose.pose.orientation.x;
  orienty_hauler_ = msg->pose.pose.orientation.y;
  orientz_hauler_ = msg->pose.pose.orientation.z;
  orientw_hauler_ = msg->pose.pose.orientation.w;
}

void Manipulation::laserCallbackHauler(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  std::vector<float> ranges = msg->ranges;
  std::sort (ranges.begin(), ranges.end());
  float min_range = 0;
  for (int i = 0; i < LASER_SET_SIZE; ++i)
  {
    min_range = min_range + ranges[i];
  }
  min_range = min_range/LASER_SET_SIZE;
  ROS_INFO_STREAM_THROTTLE(10,"HAULER: Minimum range average: " << min_range);

  if (min_range < LASER_THRESH)
  {
    ROS_INFO_THROTTLE(5,"HAULER: Close to excavator.");
    counter_laser_collision_++;
  }

  double dx = (posx_hauler_ - posx_);
  double dy = (posy_hauler_ - posy_);
  double dz = (posz_hauler_ - posz_);

  double relative_range = sqrt(dx*dx + dy*dy + dz*dz);

  relative_heading_ = atan2(dy,dx) - yaw_;

  if (manipulation_enabled_ && (counter_laser_collision_ > LASER_COUNTER_THRESH || relative_range < 4.0) )
  {
    ROS_INFO_STREAM_THROTTLE(1,"HAULER: Counter laser: " << counter_laser_collision_);
    ROS_INFO_THROTTLE(5,"HAULER: In range for dropping.");
    hauler_in_range_ = true;
  }
  else
  {
    ROS_WARN_THROTTLE(5,"HAULER: Not in range for dropping yet.");
    hauler_in_range_ = false;
  }
}

void Manipulation::bucketCallback(const srcp2_msgs::ExcavatorMsg::ConstPtr &msg)
{
  mass_in_bucket_ = msg->mass_in_bucket;

  if (mass_in_bucket_ != 0)
  {
    if (!isBucketFull_)
    {
      mass_collected_ = mass_in_bucket_ + mass_collected_;
      ROS_INFO_STREAM("Total mass collected: " << mass_collected_);
      getForwardKinematics();
      updateLocalization();
      found_volatile_ = true;
      if (optimization_counter_<3)
      {
        ROS_INFO_STREAM("A Pokemon is on the hook! Mass: " << mass_in_bucket_);
        mass_optimization_[optimization_counter_] = mass_in_bucket_;
        // ROS_INFO_STREAM("Mass optimization first dig: " << mass_optimization_[0]);
        // ROS_INFO_STREAM("Mass optimization second dig: " << mass_optimization_[1]);
        // ROS_INFO_STREAM("Mass optimization third dig: " << mass_optimization_[2]);
      }
    }
    isBucketFull_ = true;
  }
  else
  {
    isBucketFull_ = false;
  }
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
      executeHomeArm(10);
      manipulation_enabled_ = false;
    }
    break;
  case HOME_MODE:
    {
      executeHomeArm(10);
      ros::Duration(3).sleep();
    }
    break;
  case DIG_MODE:
    {
      executeDig(5);
      ros::Duration(3).sleep();
    }
    break;
  case SCOOP_MODE:
    {
      getForwardKinematics();
      executeScoop(3);
      ros::Duration(3).sleep();
      executeAfterScoop(3);
    }
    break;
  case EXTEND_MODE:
    {
      executeExtendArm(10);
      ros::Duration(3).sleep();
    }
    break;
  case DROP_MODE:
    {
      executeDrop(3);
      ros::Duration(3).sleep();
    }
    break;
  case START:
    {
      manipulation_enabled_ = true;
      found_volatile_ = false;
      mass_collected_ = 0;
      optimization_counter_ = 0;
      scoop_counter_ = 0;
      manipulation_start_time_ = ros::Time::now();
      mass_optimization_[0] = 0.0;
      mass_optimization_[1] = 0.0;
      mass_optimization_[2] = 0.0;
      counter_laser_collision_ = 0.0;
      ROS_WARN("MANIPULATION: has started!");
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
  geometry_msgs::Pose eePose_bodyframe;
  eePose_bodyframe = srv.response.eePose;

  // TODO: INCLUDE FULL ROTATION MATIRX
  eePose_.position.x = (eePose_bodyframe.position.x * cos(yaw_) - eePose_bodyframe.position.y * sin(yaw_)) + posx_;
  eePose_.position.y = (eePose_bodyframe.position.x * sin(yaw_) + eePose_bodyframe.position.y * cos(yaw_)) + posy_;
  eePose_.position.z = eePose_bodyframe.position.z + posz_;

  eePose_.orientation = eePose_bodyframe.orientation;

  ROS_INFO_STREAM("MANIPULATION: EE global position using FK: " << eePose_.position);

  double q2 = eePose_.orientation.x;
  double q3 = eePose_.orientation.y;
  double q4 = eePose_.orientation.z;
  double q1 = eePose_.orientation.w;

  double r, p, y;

  tf2::Quaternion quat(q2, q3, q4, q1); //or_x,or_y,or_z, and or_w are orientation message from nav_msg
  tf2::Matrix3x3 m(quat); // q is your quaternion message, dont take just q, but normalize it with q.normalize()
  m.getRPY(r, p, y); //get your roll pitch yaw from the quaternion message.
}

void Manipulation::updateLocalization()
{
  geometry_msgs::Pose msg;

  msg.position.x = x_goal_ - eePose_.position.x;
  msg.position.y = y_goal_ - eePose_.position.y;
  msg.position.z = z_goal_ - eePose_.position.z;
  msg.orientation.x = mass_in_bucket_;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 0.0;

  pubMeasurementUpdate.publish(msg);
  ROS_INFO_STREAM("MANIPULATION: Localization. Correction position: " << msg.position);
}


void Manipulation::getRelativePosition()
{


  // ROS_INFO_STREAM("Hauler odometry updated. Pose:" << msg->pose.pose);
  // ROS_INFO_STREAM("Range:" << relative_range);
  // ROS_INFO_STREAM("Is Hauler in range:" << hauler_in_range_);
  // ROS_INFO_STREAM("Relative heading:" << yaw_);
}

void Manipulation::executeHomeArm(double timeout)
{
  move_excavator::HomeArm srv;
  motion_control::JointGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = 0;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientHomeArm.call(srv);
}

void Manipulation::executeDig(double timeout)
{
  move_excavator::DigVolatile srv;
  motion_control::JointGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  if(found_volatile_)
  {
    ROS_INFO_STREAM("MANIPULATION: Volatile has been found");
    if (optimization_counter_<2)
    {
      volatile_heading_ = heading_options_[scoop_counter_-1] + optimization_options_[optimization_counter_];
      // ROS_INFO_STREAM("Optimization counter. Step: " << optimization_counter_);
    }
    else
    {
      // ROS_INFO_STREAM("I have three points for optimization.");
      if (mass_optimization_[1] > mass_optimization_[0] && mass_optimization_[1] > mass_optimization_[2])
      {
        // ROS_INFO_STREAM("Left point had more mass.");
        volatile_heading_ = heading_options_[scoop_counter_-1] + optimization_options_[0];
      }
      else if (mass_optimization_[2] > mass_optimization_[1] && mass_optimization_[2] > mass_optimization_[0])
      {
        // ROS_INFO_STREAM("Right point had more mass.");
        volatile_heading_ = heading_options_[scoop_counter_-1] + optimization_options_[1];
      }
      else
      {
        // ROS_INFO_STREAM("Center point had more mass.");
        volatile_heading_ = heading_options_[scoop_counter_-1];
      }
    }
    optimization_counter_++;
  }
  else
  {
    ROS_INFO_STREAM("MANIPULATION: Volatile not found yet");
    volatile_heading_ = heading_options_[scoop_counter_-1];
  }

  ROS_INFO_STREAM("MANIPULATION: Heading sent for digging: " << volatile_heading_*180/M_PI);

  srv.request.heading = volatile_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;

  bool success = clientDigVolatile.call(srv);
}

void Manipulation::executeScoop(double timeout)
{
  move_excavator::Scoop srv;
  motion_control::JointGroup q;
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
  motion_control::JointGroup q;
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
  motion_control::JointGroup q;
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
  motion_control::JointGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.heading = relative_heading_;
  srv.request.timeLimit = timeout;
  srv.request.joints = q;
  
  bool success = clientDropVolatile.call(srv);
}

void Manipulation::outputManipulationStatus()
{
  move_excavator::ExcavationStatus msg;
  msg.isFinished = true;
  msg.foundVolatile = found_volatile_;
  msg.collectedMass = mass_collected_;
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
            manipulation.executeHomeArm(10);
            if(manipulation.scoop_counter_ > 4)
            {
              manipulation.manipulation_enabled_ = false;
              manipulation.outputManipulationStatus();
              manipulation.found_volatile_ = false;
              manipulation.scoop_counter_ = 0;
              ROS_ERROR_STREAM("MANIPULATION: interrupted by scoop counter.");
            }
            else
            {
              if(manipulation.isBucketFull_)
              {
                manipulation.mode = EXTEND_MODE;
                // ROS_INFO_STREAM("Is bucket full: " << manipulation.isBucketFull_);
              }
              else
              {
                manipulation.mode = DIG_MODE;
                // ROS_INFO_STREAM("Is bucket full: " << manipulation.isBucketFull_);
              }
            }
          }
          break;
        case DIG_MODE:
          {
            if (!manipulation.found_volatile_)
            {
              manipulation.scoop_counter_++;
            }
            ROS_INFO_STREAM("MANIPULATION: Scoop counter: " << manipulation.scoop_counter_);
            manipulation.executeDig(5);
            manipulation.mode = SCOOP_MODE;
          }
          break;
        case SCOOP_MODE:
          {
            manipulation.executeScoop(0);
            ros::Time start_time = ros::Time::now();
            ros::Rate scoop_rate(100);
            while(ros::Time::now() - start_time < ros::Duration(3))
            {
              ros::spinOnce();
              rate.sleep();
            }
            manipulation.executeAfterScoop(2);
            manipulation.mode = HOME_MODE;
          }
          break;
        case EXTEND_MODE:
          {
            manipulation.executeExtendArm(10);
            ros::Duration(5).sleep();
            if(manipulation.hauler_in_range_)
            {
              // Get relative position
              manipulation.getRelativePosition();
              manipulation.mode = DROP_MODE;
            }
            else
            {
              /* code */
            }
            // manipulation.mode = DROP_MODE;
          }
          break;
        case DROP_MODE:
          {
            manipulation.executeDrop(3);
            if(abs(100-manipulation.mass_collected_)<manipulation.remaining_mass_thres_)
            {
              manipulation.manipulation_enabled_ = false;
              manipulation.executeHomeArm(10);
              manipulation.outputManipulationStatus();
              manipulation.found_volatile_ = false;
              manipulation.scoop_counter_ = 0;
              ROS_ERROR_STREAM("MANIPULATION: finished after getting the volatile.");
            }
            ros::Duration(5).sleep();
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
        manipulation.scoop_counter_ = -1;
        ROS_ERROR_STREAM("MANIPULATION: interrupted by timeout.");
      }
      else
      { 
        manipulation.manipulation_enabled_ = false;
        manipulation.found_volatile_ = false;
        manipulation.scoop_counter_ = -1;
        ROS_ERROR_THROTTLE(30,"MANIPULATION: disabled.");
      }
      

      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
