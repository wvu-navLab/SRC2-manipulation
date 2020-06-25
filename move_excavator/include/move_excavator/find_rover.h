/*!
 * \find_rover.h
 * \brief Find SRC2 Rover using Camera and Lidar
 *
 * Find Rover creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * * \date April 28, 2020
 */

#ifndef FIND_ROVER_H
#define FIND_ROVER_H

#include <math.h>
#include <stdio.h> 

// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <move_excavator/MultiAgentState.h>

#define PI 3.141592653589793
class FindRover
{
public:
    FindRover(ros::NodeHandle & nh);
    ~FindRover();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;

    // Node Handle
    ros::NodeHandle & nh_;

    // Subscribers
    ros::Publisher pubMultiAgentState;

    // Subscribers
    ros::Subscriber subLaserScan;
    message_filters::Subscriber<sensor_msgs::Image> subImgNoSync;

    // Service Callers
    ros::ServiceServer stopServer;
    ros::ServiceServer rotateInPlaceServer;

    move_excavator::MultiAgentState m;
};

#endif // ROTATE_IN_PLACE_H