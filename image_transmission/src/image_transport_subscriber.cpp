/*
 *  Created on: Jan 8, 2012
 *      Author: Kai Franke
 */
 
#include "image_transport_subscriber.h"

cv_bridge::CvImagePtr cv_ptr;
unsigned int img_counter;
ros::Time start;
ros::Time stop;
bool reset;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_transport_subscriber");
  image_transport_subscriber its;
  reset=true;
  ROS_INFO("waiting for 30 images");
  ros::spin();
  return 0;
}

image_transport_subscriber::image_transport_subscriber():it_(nh_)
{	
	ROS_INFO("constructed");	
  	image_sub = it_.subscribe("/image_benchmark", 30, &image_transport_subscriber::imageCb, this);
	img_counter = 0;
}

// Destructor
image_transport_subscriber::~image_transport_subscriber()
{
	cv::destroyAllWindows();
}

// reads an image from the subscribed topic
void image_transport_subscriber::imageCb(const sensor_msgs::ImageConstPtr& imgMsgPtr)
{	
	if(reset)
	{
		start = ros::Time::now();
		reset = false;
	}
	// Convert image to openCV datatype
	try
	{
		cv_ptr = cv_bridge::toCvCopy(imgMsgPtr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	ROS_DEBUG("Received Image %u",imgMsgPtr->header.seq);
	
	if(++img_counter >=30)
	{
		stop = ros::Time::now();
		img_counter = 0;
		reset = true;
		ROS_DEBUG("Received 30 images within %llu milliseconds", (stop.toNSec() -start.toNSec())/1000000);
		// output Milliseconds for easy dopy/paste
		cout << (stop.toNSec() -start.toNSec())/1000000 << '\n';
	}
}


