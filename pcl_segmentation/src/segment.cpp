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

using namespace std;

// Initialize ROS
ros::Publisher pub;

// construct point cloud to work with
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// construct coefficients
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

// constructor for point found as part of planar surface
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

// Create the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

// Create writer to write PCD files
pcl::PCDWriter writer;

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

	// measure time to find biggest planar surface
	start = ros::Time::now();
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;
	
	// measure time for overwriting filtered values
	start = ros::Time::now();
	extract.setIndices (inliers);
	extract.filterDirectly (cloud);
	stop = ros::Time::now();
	cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;

	// output number of points found on a surface
	cout << inliers->indices.size() << endl;

	writer.write<pcl::PointXYZ> ("no_table.pcd", *cloud, false);
}

int
main (int argc, char** argv)
{
	ros::init (argc, argv, "segment");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("pointcloud", 1, cloud_cb);

	// Set up SAC parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	// maximal distance from point to planar surface to be identified as plane (1cm???)
	seg.setDistanceThreshold (0.05);
	// limit maximum computation time
	seg.setMaxIterations (1000);
	
	// Extract the outliers (not the planar surface)
	extract.setNegative (true);

	// Write description for csv file	
	cout << "time[ms];matching points" << endl;
	
	// Spin
	ros::spin ();
}
