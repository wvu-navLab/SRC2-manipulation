

#ifndef FIND_EXCAVATOR_H
#define FIND_EXCAVATOR_H

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
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <move_excavator/FindExcavator.h>
#include <srcp2_msgs/SpotLightSrv.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


class FindExcavator
{
public:
    FindExcavator(ros::NodeHandle & nh);
    ~FindExcavator();
    //void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msgl, const sensor_msgs::CameraInfoConstPtr& info_msgl, const sensor_msgs::ImageConstPtr& msgr, const sensor_msgs::CameraInfoConstPtr& info_msgr);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    
    bool FindExcavatorService(move_excavator::FindExcavator::Request  &req, move_excavator::FindExcavator::Response &res);
    
    bool ComputeExcavatorPosition();
    
    static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
    void CallBackFunc(int event, int x, int y, int flags);

private:
    
    // Node Handle
    ros::NodeHandle & nh_;

    // Subscribers
    ros::Publisher pubSensorYaw;

    // Subscribers
    //ros::Subscriber subLaserScan;
    ros::Subscriber subJointStates;
   
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
    
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync; 
    
    // Service Servers
    ros::ServiceServer serverFindExcavator; 
    
    ros::ServiceClient clientSpotLight;
    
    // Service Callers
    //ros::ServiceServer stopServer;
    //ros::ServiceServer rotateInPlaceServer;
    
    geometry_msgs::PointStamped target_;
    
    int ximg,yimg;
    
    int iLowH_, iHighH_, iLowS_, iHighS_, iLowV_, iHighV_;
    
    double currSensorYaw_;
    
    int direction_;
    
    cv::Mat raw_imagel_, raw_imager_;
    sensor_msgs::CameraInfo info_msgl_, info_msgr_;
    double x_, y_, z_;
    
    bool static compareKeypoints(const cv::KeyPoint &k1, const cv::KeyPoint &k2);
};

#endif // FIND_EXCAVATOR_H
