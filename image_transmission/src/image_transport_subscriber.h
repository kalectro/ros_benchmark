/*
 *  Created on: Jan 08, 2013
 *      Author: Kai Franke
 *
 *  Receives an image from a topic and saves it to IMAGE_PATH
 */

#ifndef IMAGETRANSPORTRECEIVER_STD_H_
#define IMAGETRANSPORTRECEIVER_STD_H_


#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;

int main(int, char**);

class image_transport_subscriber
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub;

public:
  image_transport_subscriber();
  virtual ~image_transport_subscriber();

  // Receive the image and coordinate the further processing
  void imageCb(const sensor_msgs::ImageConstPtr& imgMsgPtr);

};


#endif
