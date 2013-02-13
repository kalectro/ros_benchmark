#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>

using namespace std;

// Initialize ROS
ros::Publisher pub;

// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

// Create the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// declare timing variables
	ros::Time start, stop;	

	// measure time for converting the message
	start = ros::Time::now();
	pcl::fromROSMsg (*input, *cloud);
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;

	// measure time segment pointcloud
	start = ros::Time::now();
	seg.segment (*inliers, *coefficients);
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;
	
	// measure time for making input cloud accessible
	start = ros::Time::now();
	extract.setIndices (inliers);
	extract.filterDirectly (cloud);
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;

	// output number of points found on a surface
	cout << inliers->indices.size() << endl;
	
	// Publish the model coefficients
	pub.publish (coefficients);
}

int
main (int argc, char** argv)
{
	ros::init (argc, argv, "segment");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("pointcloud", 1, cloud_cb);
	
	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<pcl::ModelCoefficients> ("coefficients", 1);

	// Set up SAC parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	// maximal distance from point to planar surface to be identified as plane (1cm???)
	seg.setDistanceThreshold (0.01);
	// limit maximum computation time
	seg.setMaxIterations (1000);
	
	// Extract the outliers (not the planar surface)
	extract.setIndices (inliers);
	extract.setNegative (false);

	// Write description for csv file	
	cout << "time[ms];matching points" << endl;
	
	// Spin
	ros::spin ();
}
