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

FindRover::FindRover(ros::NodeHandle & nh)
: nh_(nh),
subImgNoSync(nh_, "camera/left/image_raw", 1)
{
  
  cv::namedWindow("original");
  cv::namedWindow("blobs");
  cv::startWindowThread(); 
  
  // Node publishes individual joint positions
  pubMultiAgentState = nh_.advertise<move_excavator::MultiAgentState>("/multiAgent", 1000);

  subImgNoSync.registerCallback(&FindRover::imageCallback, this);
  subLaserScan = nh_.subscribe("laser/scan", 1000, &FindRover::laserCallback, this);
}

FindRover::~FindRover()
{
  cv::destroyAllWindows();
}

void FindRover::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
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
  ROS_INFO_STREAM("Minimum distance in Lidar view: " << msg->ranges[closestIndex]);

  if(msg->ranges[closestIndex] < 0.8)
  {
    m.isRoverInRange = true;
    pubMultiAgentState.publish(m);
  }
  else
  {
    m.isRoverInRange = false;
    pubMultiAgentState.publish(m);
  }
}

void FindRover::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  int iLowH = 15;
  int iHighH = 35;

  int iLowS = 60;
  int iHighS = 255;

  int iLowV = 100;
  int iHighV = 255;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow("original", cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat raw_image = cv_ptr->image;
  cv::Mat hsv_image;
  cv::cvtColor(raw_image, hsv_image,CV_BGR2HSV);
  
  cv::Mat imgThresholded;
  cv::inRange(hsv_image, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); 

  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Filter by Area.
  params.filterByColor = true;
  params.blobColor = 255;

  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 200;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.1;

  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.1;

  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;
  
  // SimpleBlobDetector::create creates a smart pointer.
  // Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  // Detect blobs.
  std::vector<cv::KeyPoint> keypoints;
  detector->detect( imgThresholded, keypoints);

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  cv::Mat im_with_keypoints;
  cv::drawKeypoints( imgThresholded, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  if (keypoints.size() != 0)
  {
    m.isRoverInView = true;
    pubMultiAgentState.publish(m);
  }
  else
  {
    m.isRoverInView = false;
    pubMultiAgentState.publish(m);
  }
  

  // Show blobs
  imshow("blobs", im_with_keypoints);
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