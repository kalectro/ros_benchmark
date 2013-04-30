#include "image_transport_publisher.h"

using namespace std;
//namespace enc = sensor_msgs::image_encodings;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_transport_publisher");
	ros::NodeHandle nh("~");
	ros::Time start;
	ros::Time stop;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("/image_benchmark", 2);

	// Give advertise some time
	sleep(1);

	// get parameters from server
	if (nh.getParam("image_path", image_path))
	{
		ROS_INFO("Found image path %s on server", image_path.c_str());
	}
	else
	{
		image_path = IMAGE_PATH;
		ROS_INFO("Found no image path on server, using default %s", image_path.c_str());
	}

	// Create new CvImage object
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

	ROS_INFO("loading pictures");
	cv_image  = cv::imread(image_path, 1); // Read RGB image

  // check if image was loaded successfully
	if (!cv_image.data)
	{
		ROS_ERROR("Could not open or find image at file path %s", image_path.c_str());
		ROS_ERROR("Set parameter image_path for file path to image");
		nh.shutdown();
		return -1;
	}

  // assign OpenCV image to cv_bridge object
	cv_ptr->image = cv_image;

	// Write msg data
	cv_ptr->encoding = "bgr8"; // image encoding
	
  while (nh.ok())
  {
	  // start timer
	  start = ros::Time::now();
	  for(int i=0;i<30;++i)
	  {
	    // create image message and publish
	    image_pub.publish(cv_ptr->toImageMsg());
	    ROS_DEBUG("Published 1 image");
	  }
	  stop = ros::Time::now();
	  
	  // output time it took to transmit 30 images
	  cout << (stop-start).toNSec()/1000000 <<'\n';
  }
  
	return 0;
}
