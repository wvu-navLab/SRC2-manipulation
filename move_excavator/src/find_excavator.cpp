

#include "move_excavator/find_excavator.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//l#define SHOWIMG
#define MIN_AREA 300

void FindExcavator::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
}


FindExcavator::FindExcavator(ros::NodeHandle & nh)
: nh_(nh), sync(MySyncPolicy(10), left_image_sub, left_info_sub, right_image_sub, right_info_sub)
{
  ximg=0;
  yimg=0;
#ifdef SHOWIMG  
  cv::namedWindow("originall");
  cv::startWindowThread(); 
  cv::setMouseCallback("originall", CallBackFunc, this);
#endif  
  
  
  pubSensorYaw = nh_.advertise<std_msgs::Float64>("sensor/yaw/command/position", 1000);

  // Service Servers
  serverFindExcavator= nh_.advertiseService("manipulation/find_excavator", &FindExcavator::FindExcavatorService, this);
  
  // Service Clients
  clientSpotLight = nh_.serviceClient<srcp2_msgs::SpotLightSrv>("spot_light");
  
  
  subJointStates = nh_.subscribe("joint_states", 1, &FindExcavator::jointStateCallback, this);
  
  right_image_sub.subscribe(nh_,"camera/right/image_raw", 1);
  left_image_sub.subscribe(nh_,"camera/left/image_raw", 1);
  right_info_sub.subscribe(nh_,"camera/right/camera_info", 1);
  left_info_sub.subscribe(nh_,"camera/left/camera_info", 1);
  
  sync.registerCallback(boost::bind(&FindExcavator::imageCallback,this, _1, _2, _3, _4));
  
  iLowH_ = 0;
  iHighH_ = 5;

  iLowS_ = 130;
  iHighS_ = 250;

  iLowV_ = 100;
  iHighV_ = 255;
  
  currSensorYaw_=0.0;
  direction_ = 1;
  
}

FindExcavator::~FindExcavator()
{
  cv::destroyAllWindows();
}



bool FindExcavator::compareKeypoints(const cv::KeyPoint &k1, const cv::KeyPoint &k2)
{
	if (k1.size > k2.size) return true;
  	else return false;
}

void FindExcavator::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  int sensor_bar_yaw_joint_idx;

  // loop joint states
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "sensor_bar_yaw_joint") {
      sensor_bar_yaw_joint_idx = i;
    }
  }

 currSensorYaw_ = msg->position[sensor_bar_yaw_joint_idx];  
}

bool FindExcavator::FindExcavatorService(move_excavator::FindExcavator::Request  &req, move_excavator::FindExcavator::Response &res)
{

  ROS_INFO("Service FindExcavatorCalled");
  //turn on the light
  srcp2_msgs::SpotLightSrv srv;
  srv.request.range = 20;
  clientSpotLight.call(srv);
  
  ros::Duration timeout(req.timeLimit);
	
  cv::Mat hsv_imagel;
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Filter by Area.
  params.filterByColor = false;
  params.blobColor = 255;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = MIN_AREA;
  params.maxArea = 2000000;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.1;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;
  
  // SimpleBlobDetector::create creates a smart pointer.
  // Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  // Detect blobs.
  std::vector<cv::KeyPoint> keypointsl, keypointsr;
  
  std_msgs::Float64 nextAngle;
  
  long int currentFrameId;
 
  ros::Time start_time = ros::Time::now();

  int previous_direction = 2;

  direction_ = req.side;

 
  if (direction_==0) direction_=-1;

  
  do
  {

    //ROS_INFO("Current Sensor Yaw: %f", currSensorYaw_);

    ros::spinOnce();
    currentFrameId = target_.header.seq;
    cv::cvtColor(raw_imagel_, hsv_imagel, CV_BGR2HSV);
    // cv::imshow("hsv_imagel", hsv_imagel);

    cv::Mat imgThresholdedl;
    cv::inRange(hsv_imagel, cv::Scalar(iLowH_, iLowS_, iLowV_), cv::Scalar(iHighH_, iHighS_, iHighV_), imgThresholdedl);
    cv::dilate(imgThresholdedl, imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 2);
    cv::erode(imgThresholdedl, imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 3);

    detector->detect(imgThresholdedl, keypointsl);

    // Draw detected blobs as red circles.

    #ifdef SHOWIMG
        cv::Mat im_with_keypointsl;
        cv::drawKeypoints(imgThresholdedl, keypointsl, im_with_keypointsl, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow("blobsl", im_with_keypointsl);
    #endif

    if (keypointsl.size() > 0)
    {
      // Sort the keypoints in order of area
      std::sort(keypointsl.begin(), keypointsl.end(), compareKeypoints);
    
      int i = 0; // Get the larger keypoints in each camera

      // Error to the center of the image
      double error = (raw_imagel_.cols) / 2.0 - keypointsl[i].pt.x;
      //ROS_INFO("Error %f, Center: %f, Keypoint: %f", error, (raw_imagel_.cols)/2.0, keypointsl[i].pt.x );

      if ((fabs(error) < 5.0) || (currSensorYaw_ < -(M_PI /6.0)) || (currSensorYaw_ > (M_PI / 6.0)))
      {

        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if (ComputeExcavatorPosition())
        {
          res.success = true;
          res.target = target_;
          nextAngle.data = 0.0;
          pubSensorYaw.publish(nextAngle);
          return true;
        }
        else
        {
          res.success = false;
          nextAngle.data = 0.0;
          pubSensorYaw.publish(nextAngle);
          return true;
        }
      }
      
      nextAngle.data = currSensorYaw_ + sgn(error) * M_PI / (90.0);
      pubSensorYaw.publish(nextAngle);
      ros::Duration(0.1).sleep();
      direction_ = sgn(error); // force search if loose target
      previous_direction = -sgn(error); 
    }
    else
    {
    
      if (currSensorYaw_ < -(M_PI /6.0) && (previous_direction == -1))
        direction_ = 1;
      if (currSensorYaw_ > (M_PI / 6.0) && (previous_direction == 1))
        direction_ = -1;

      if(direction_ != previous_direction)
      {
        nextAngle.data = direction_ * M_PI;
        //ROS_INFO("%f %f %d", nextAngle.data, currSensorYaw_, direction_);

        pubSensorYaw.publish(nextAngle);
        ros::Duration(0.1).sleep();
        previous_direction = direction_;
      }
      
    }
    // Wait for new frame -> this is important to free the processor
    while ((currentFrameId == target_.header.seq) && ((ros::Time::now() - start_time) < timeout))
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }

  } while ((ros::Time::now() - start_time) < timeout);

  nextAngle.data = 0.0;
  pubSensorYaw.publish(nextAngle);
  ROS_INFO("FindExcavator: TimeOut");

  res.success = false;
  return true;
}

void FindExcavator::imageCallback(const sensor_msgs::ImageConstPtr& msgl, const sensor_msgs::CameraInfoConstPtr& info_msgl, const sensor_msgs::ImageConstPtr& msgr, const sensor_msgs::CameraInfoConstPtr& info_msgr)
{
  

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgl, sensor_msgs::image_encodings::BGR8);
    #ifdef SHOWIMG     
        cv::imshow("originall", cv_ptr->image);
    #endif    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  raw_imagel_ = cv_ptr->image;
  info_msgl_ = *info_msgl;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgr, sensor_msgs::image_encodings::BGR8);
#ifdef SHOWIMG         
    cv::imshow("original1", cv_ptr->image);
#endif    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  raw_imager_ = cv_ptr->image;
  info_msgr_ = *info_msgr;
  
  target_.header = msgl->header;

  
  ////////////////////////////////////////
  
}
      
      
bool FindExcavator::ComputeExcavatorPosition()
{    
  cv::Mat hsv_imagel;
  cv::cvtColor(raw_imagel_, hsv_imagel, CV_BGR2HSV);
 // cv::imshow("hsv_imagel", hsv_imagel);
  
  cv::Mat hsv_imager;
  cv::cvtColor(raw_imager_, hsv_imager, CV_BGR2HSV);
 // cv::imshow("hsv_imager", hsv_imager);
   
  //cv::Vec3b hsvPixel = hsv_imagel.at<cv::Vec3b>(280,484);
  //ROS_INFO("%d %d %d", hsvPixel.val[0], hsvPixel.val[1], hsvPixel.val[2] );
  
  cv::Mat imgThresholdedl;
  cv::inRange(hsv_imagel, cv::Scalar(iLowH_, iLowS_, iLowV_), cv::Scalar(iHighH_, iHighS_, iHighV_), imgThresholdedl); 
  cv::dilate(imgThresholdedl,imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(imgThresholdedl,imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 3);
  
  cv::Mat imgThresholdedr;
  cv::inRange(hsv_imager, cv::Scalar(iLowH_, iLowS_, iLowV_), cv::Scalar(iHighH_, iHighS_, iHighV_), imgThresholdedr); 
  cv::dilate(imgThresholdedr,imgThresholdedr, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(imgThresholdedr,imgThresholdedr, cv::Mat(), cv::Point(-1, -1), 3);
  
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Filter by Area.
  params.filterByColor = false;
  params.blobColor = 255;

  // Change thresholds
  //params.minThreshold = 10;
  //params.maxThreshold = 200;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = MIN_AREA;
  params.maxArea = 2000000;

  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.1;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;
  
  // SimpleBlobDetector::create creates a smart pointer.
  // Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  // Detect blobs.
  std::vector<cv::KeyPoint> keypointsl, keypointsr;
  detector->detect( imgThresholdedl, keypointsl);
  detector->detect( imgThresholdedr, keypointsr);

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
#ifdef SHOWIMG  
  cv::Mat im_with_keypointsl, im_with_keypointsr; 
  cv::drawKeypoints( imgThresholdedl, keypointsl, im_with_keypointsl, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  cv::drawKeypoints( imgThresholdedr, keypointsr, im_with_keypointsr, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
#endif

#ifdef SHOWIMG 
  // Show blobs
  imshow("blobsl", im_with_keypointsl);
  imshow("blobsr", im_with_keypointsr);
#endif  
  
  if ((keypointsl.size() > 0) && (keypointsr.size() > 0))
  {
 	// Sort the keypoints in order of area
 	std::sort(keypointsl.begin(), keypointsl.end(), compareKeypoints);
 	std::sort(keypointsr.begin(), keypointsr.end(), compareKeypoints);
 	
 	/*for (int i=0;i<keypointsl.size();i++){
 		ROS_INFO("Left: Area %f", keypointsl[i].size);
 	}
 	for (int i=0;i<keypointsr.size();i++){
 		ROS_INFO("Righ: Area %f", keypointsr[i].size);
 	}
 	ROS_INFO("____");*/
 	
 	
 	int i=0; // Get the larger keypoints in each camera
 	
  //ROS_INFO("%f", fabs(keypointsl[i].size - keypointsr[i].size));
 	// check if is the same object 
 	if (fabs(keypointsl[i].size - keypointsr[i].size)<100)
 	{
 		//ROS_INFO("Pass 1");
    double disparity = keypointsl[i].pt.x - keypointsr[i].pt.x;
 		double offset = keypointsl[i].pt.y - keypointsr[i].pt.y;
    //ROS_INFO("%f %f", disparity, offset); 	
 		if(disparity > 5 && disparity < 180)
		{
      //ROS_INFO("Pass 2");
      //check epipolar constraint
     	if(offset < 10 && offset > -10)
			{
        //ROS_INFO("Pass 3");
        double cx = (double) info_msgl_.P[2];
				double cy = (double) info_msgl_.P[6];
				double sx = (double) info_msgl_.P[0];
				double sy = (double) info_msgl_.P[5];
				double bl = (double) (-(double)info_msgr_.P[3]/info_msgr_.P[0]);

				z_ = sx/disparity*bl;
				x_ = (keypointsl[i].pt.x-cx)/sx*z_;
				y_ = (keypointsl[i].pt.y-cy)/sy*z_;
			
				//ROS_INFO("(%f,%f,%f)", x_, y_, z_);
				target_.point.x = x_;
        target_.point.y = y_;
        target_.point.z = z_;
        //pubTarget.publish(target_);

        return true;
			}
		}
	}
  }

return false;
}

/*!
 * \brief Creates and runs the FindExcavator node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_excavator");
  ros::NodeHandle nh("");
  ros::Rate rate(50);

  ROS_INFO("Find Excavator Node initializing...");
  FindExcavator find_rover(nh);

  while(ros::ok()) 
  {
		ros::spinOnce();
		rate.sleep();
	}
  return 0;
}
