/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Jan 15, 2013
 *      Author: Kai Franke
 */


#include <feature_detection.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_detection");
	ros::NodeHandle nh("~");
	// variables for timing
	ros::Time start;
	ros::Time stop;
	
	// Debug publisher
	// ros::Publisher features_pub = nh.advertise<cv::Mat>("/image_benchmark/features", 100);

	// Init nonfree feature detection from OpenCV2
	initModule_nonfree(); 

	// get parameters from server
	// get image path
	if (nh.getParam("image_path", image_path))
	{
		ROS_DEBUG("Found image path %s on server", image_path.c_str());
	}
	else
	{
		image_path = IMAGE_PATH;
		ROS_WARN("Found no image path on server, using default %s", image_path.c_str());
	}

	// get feature detector type
	nh.param<std::string>("detector_type", detector_type, "FAST");

	// get feature extractor type
	nh.param<std::string>("extractor_type", extractor_type, "BRISK");
		
	// get verbosity for csv output
	nh.param<bool>("verbosity", verbosity, false);
	
	// get number of repetitions
	nh.param<int>("repetitions", repetitions, 1);

	// construct feature detector and descriptor extractor
	Ptr<FeatureDetector> detector = FeatureDetector::create(detector_type);
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(extractor_type);

	// set the header line for a cvs output seperated by ; if verbose
	if(verbosity)
		cout << "detection[ms];keypoints;detector;extraction;keypoints;image path;extractor\n";

	ROS_DEBUG("loading picture");
	cv_image  = cv::imread(image_path, 1); // Read RGB image

	if (!cv_image.data)
	{
		ROS_ERROR("Could not open or find image at file path %s", image_path.c_str());
		ROS_ERROR("Set parameter image_path for file path to image");
		nh.shutdown();
		return -1;
	}

	// some detectors will take longer on their first feature detection
	// to simulate detecting a stream of images, detection and extraction are called here without timing
	detector->detect(cv_image, keypoints);
	extractor->compute(cv_image, keypoints, descriptors);

	for (int i=0;i<repetitions;++i)
	{	
		// get parameters from server
		// get image path
		if (nh.getParam("image_path", image_path))
		{
			ROS_DEBUG("Found image path %s on server", image_path.c_str());
		}
		else
		{
			image_path = IMAGE_PATH;
			ROS_DEBUG("Found no image path on server, using default %s", image_path.c_str());
		}

		// get feature detector type
		nh.param<std::string>("detector_type", detector_type, "FAST");

		// get feature extractor type
		nh.param<std::string>("extractor_type", extractor_type, "BRISK");

		// start timer
		start = ros::Time::now();
		// Detect feature points
		detector->detect(cv_image, keypoints);
		// stop the timer
		stop = ros::Time::now();
		// output time for detector
		cout << (stop-start).toNSec()/1000000 << ';' << keypoints.size() << ';' << detector_type << ';' ;
	
		// start timer
		start = ros::Time::now();
		// Get descriptors for keypoints
		extractor->compute(cv_image, keypoints, descriptors);
		// stop the timer
		stop = ros::Time::now();
		// output time for feature extractor
		cout << (stop-start).toNSec()/1000000 << ';' << keypoints.size() << ';' << image_path << ';' << extractor_type << '\n';
	}
	return 0;
}
