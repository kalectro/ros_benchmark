/*
 *  Created on: Jan 8, 2012
 *      Author: Kai Franke
 */
 
#include "image_transport_subscriber.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_transport_subscriber");
  image_transport_subscriber its;
  ROS_INFO("waiting for 30 images");
  ros::spin();
  return 0;
}

image_transport_subscriber::image_transport_subscriber():it_(nh_)
{	
  image_sub_ = it_.subscribe("/image_benchmark", 30, &image_transport_subscriber::imageCb, this);
  img_counter_ = 0;
}

// Destructor
image_transport_subscriber::~image_transport_subscriber()
{
}

// reads an image from the subscribed topic
void image_transport_subscriber::imageCb(const sensor_msgs::ImageConstPtr& imgMsgPtr)
{
  static ros::Time start;
  static ros::Time stop;
  
  // new bundle of images received
	if( img_counter_ == 0 )
	{
		start = ros::Time::now();
	}
	
	// Convert image to openCV datatype
	try
	{
		cv_ptr_ = cv_bridge::toCvCopy(imgMsgPtr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if(++img_counter_ >=30)
	{
		stop = ros::Time::now();
		img_counter_ = 0;
		// output Milliseconds for easy copy/paste
		cout << ((stop - start).toNSec())/1000000 << '\n';
	}
}


