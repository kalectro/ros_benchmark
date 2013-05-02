/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

//\Author Kai Franke, Robert Bosch LLC

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


