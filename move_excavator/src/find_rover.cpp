/*!
 * \find_rover.cpp
 * \brief Algorithms for Finding Rover for SRC2 Rovers
 *
 * FindRover creates a ROS node that ...
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date May 04, 2020
 */

#include "move_excavator/find_rover.h"

//#define SHOWIMG

void FindRover::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
}


FindRover::FindRover(ros::NodeHandle & nh)
: nh_(nh), sync(MySyncPolicy(10), left_image_sub, left_info_sub, right_image_sub, right_info_sub)
{
  ximg=0;
  yimg=0;
#ifdef SHOWIMG  
  cv::namedWindow("originall");
  //cv::startWindowThread(); 
  cv::setMouseCallback("originall", CallBackFunc, this);
#endif  
  cv::startWindowThread();
  
  pubMultiAgentState = nh_.advertise<move_excavator::MultiAgentState>("/multiAgent", 1000);
  
  pubTarget = nh_.advertise<geometry_msgs::PointStamped>("manipulation/target_bin", 1000);
  
  pubSensorYaw = nh_.advertise<std_msgs::Float64>("sensor/yaw/command/position", 1000);

  // Service Servers
  serverFindHauler = nh_.advertiseService("manipulation/find_hauler", &FindRover::FindHauler, this);
  
  // Service Clients
  clientSpotLight = nh_.serviceClient<srcp2_msgs::SpotLightSrv>("spot_light");
  
  // Subscribers
  //subLaserScan = nh_.subscribe("laser/scan", 1000, &FindRover::laserCallback, this);
  
  subJointStates = nh_.subscribe("joint_states", 1, &FindRover::jointStateCallback, this);
  
  right_image_sub.subscribe(nh_,"camera/right/image_raw", 1);
  left_image_sub.subscribe(nh_,"camera/left/image_raw", 1);
  right_info_sub.subscribe(nh_,"camera/right/camera_info", 1);
  left_info_sub.subscribe(nh_,"camera/left/camera_info", 1);
  
  sync.registerCallback(boost::bind(&FindRover::imageCallback,this, _1, _2, _3, _4));
  
  iLowH_ = 0;
  iHighH_ = 5;

  iLowS_ = 130;
  iHighS_ = 250;

  iLowV_ = 100;
  iHighV_ = 255;
  
  currSensorYaw_=0.0;
  
}

FindRover::~FindRover()
{
  cv::destroyAllWindows();
}

/*void FindRover::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  int size = msg->ranges.size();
  int minIndex = 0;
  int maxIndex = size-1; 
  int closestIndex = -1;
  double minVal = 999; //values are between 0.2 and 30 meters for my scanner

  for (int i = minIndex; i < maxIndex; i++)
  {
      if ((msg->ranges[i] <= minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
      {
          minVal = msg->ranges[i];
          closestIndex = i;
      }
  }
  //ROS_INFO_STREAM("Minimum distance in Lidar view: " << msg->ranges[closestIndex]);

  if(msg->ranges[closestIndex] < 0.8)
  {
    m.isRoverInRange = true;
  }
  else
  {
    m.isRoverInRange = false;
  }
  pubMultiAgentState.publish(m);
}*/


bool FindRover::compareKeypoints(const cv::KeyPoint &k1, const cv::KeyPoint &k2)
{
	if (k1.size > k2.size) return true;
  	else return false;
}

void FindRover::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
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

bool FindRover::FindHauler(move_excavator::FindHauler::Request  &req, move_excavator::FindHauler::Response &res)
{

  ROS_INFO("Service FindHauler Called");
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
  params.minArea = 4000;
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
  
  ros::Time start_time = ros::Time::now();
  
  do{
  
  	ros::spinOnce();
  	cv::cvtColor(raw_imagel_, hsv_imagel, CV_BGR2HSV);
 	// cv::imshow("hsv_imagel", hsv_imagel);
    
  	cv::Mat imgThresholdedl;
  	cv::inRange(hsv_imagel, cv::Scalar(iLowH_, iLowS_, iLowV_), cv::Scalar(iHighH_, iHighS_, iHighV_), imgThresholdedl); 
  	cv::dilate(imgThresholdedl,imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 2);
  	cv::erode(imgThresholdedl,imgThresholdedl, cv::Mat(), cv::Point(-1, -1), 3);
   
  	detector->detect( imgThresholdedl, keypointsl);
  
  	// Draw detected blobs as red circles.
  
//#ifdef SHOWIMG  
  	cv::Mat im_with_keypointsl; 
  	cv::drawKeypoints( imgThresholdedl, keypointsl, im_with_keypointsl, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  	imshow("blobsl", im_with_keypointsl);
//#endif
	
	
	if (keypointsl.size() > 0)
  	{
 		// Sort the keypoints in order of area
 		std::sort(keypointsl.begin(), keypointsl.end(), compareKeypoints);
 	 	
 		/*for (int i=0;i<keypointsl.size();i++){
 		ROS_INFO("Left: Area %f", keypointsl[i].size);
 		}
 	
 		ROS_INFO("____");*/
 	
 	
 		int i=0; // Get the larger keypoints in each camera
 	
 	
 		double error = -keypointsl[i].pt.x;
 	
 	        res.success = true;
 		return true;
   	}
   	else 
   	{
   	       std_msgs::Float64 nextAngle;
   	       nextAngle.data=M_PI/2.0;//(currSensorYaw_+2);
   	       
               pubSensorYaw.publish(nextAngle);
               
   
   	}
   	ros::Duration(0.5).sleep();
   }	
   while ((ros::Time::now() - start_time) < timeout);
   ROS_INFO("TimeOut");
	
   res.success = false;
   return false;
   
}

void FindRover::imageCallback(const sensor_msgs::ImageConstPtr& msgl, const sensor_msgs::CameraInfoConstPtr& info_msgl, const sensor_msgs::ImageConstPtr& msgr, const sensor_msgs::CameraInfoConstPtr& info_msgr)
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
  
  target.header = msgl->header;
  
  ////////////////////////////////////////
  
      
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
  params.minArea = 4000;
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
 	
 	// check if is the same object 
 	if (fabs(keypointsl[i].size - keypointsr[i].size)<20)
 	{
 		double disparity = keypointsl[i].pt.x - keypointsr[i].pt.x;
 		double offset = keypointsl[i].pt.y - keypointsr[i].pt.y;
 	
 		if(disparity > 5 && disparity < 100)
		{
			//check epipolar constraint
     			 if(offset < 5 && offset > -5)
			{
				double cx = (double) info_msgl->P[2];
				double cy = (double) info_msgl->P[6];
				double sx = (double) info_msgl->P[0];
				double sy = (double) info_msgl->P[5];
				double bl = (double) (-(double)info_msgr->P[3]/info_msgr->P[0]);

				double z = sx/disparity*bl;
				double x = (keypointsl[i].pt.x-cx)/sx*z;
				double y = (keypointsl[i].pt.y-cy)/sy*z;
			
				//ROS_INFO("(%f,%f,%f)", x, y, z);
				target.point.x = x;
        			target.point.y = y;
        			target.point.z = z;
        			pubTarget.publish(target);
			}
		}
	}
  } 

  
  
#ifdef SHOWIMG 
  // Show blobs
  imshow("blobsl", im_with_keypointsl);
  imshow("blobsr", im_with_keypointsr);
#endif  
}

/*!
 * \brief Creates and runs the FindRover node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_rover");
  ros::NodeHandle nh("");
  ros::Rate rate(50);

  ROS_INFO("Find Rover Node initializing...");
  FindRover find_rover(nh);

  while(ros::ok()) 
  {
		ros::spinOnce();
		rate.sleep();
	}
  return 0;
}
