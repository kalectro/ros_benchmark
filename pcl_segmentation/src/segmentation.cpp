#include "segmentation.h"

using namespace std;

// Construct point cloud to work with
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// construct coefficients
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

// constructor for point found as part of planar surface
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

// Create pass through point cloud for point filtering
pcl::PassThrough<sensor_msgs::PointCloud2> pt(false);

// Create ROS message for filtered point cloud
sensor_msgs::PointCloud2 input_filtered;

// Declare the segmentation object
pcl::SACSegmentation<pcl::PointXYZ> seg;

// Declare the filtering object
pcl::ExtractIndices<pcl::PointXYZ> extract;

// Declare writer to write PCD files
pcl::PCDWriter writer;


int main (int argc, char** argv)
{
	ros::init (argc, argv, "plane_segmentation");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("pointcloud", 1, cloud_cb);

	// Write description for csv file	
	cout << "All time values measured in milli seconds" << endl;
	cout << "filter_z;ROS2PC;segment_plane;update_PC;write_output;points_on_plane" << endl;

	// Set up SAC parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	// maximal distance from point to planar surface to be identified as plane (1cm???)
	seg.setDistanceThreshold (0.02);
	// limit maximum computation time
	seg.setMaxIterations (1000);
	
	// Extract the outliers (not the planar surface)
	extract.setNegative (true);

	// get verbosity for pcd output
	nh_param.param<bool>("output_pcd", output_pcd, false);
	
	// Spin
	ros::spin ();
}

void duration(bool identifier)
{
	// 0 = STOP ; 1= START
	// start timer if START
	// stop and print timer if STOP
	if(identifier)
		start = ros::Time::now();
	else
	{
		stop = ros::Time::now();
		cout << (stop.toNSec()-start.toNSec())/1000000 << ';' ;
	}
}	

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// make node handle available for retrieving parameters
	ros::NodeHandle nh;

	// measure time for filtering the points
	// check if parameters for limiting z value exists
	if (nh.hasParam("/segment/z_min_distance") && nh.hasParam("/segment/z_max_distance"))
	{
		// get limits for filtering
		do
		{
			nh.getParam("/segment/z_min_distance", z_min);
			nh.getParam("/segment/z_max_distance", z_max);
			nh.param("/segment/keep_organized", keep_organized, false);
			// check if range is defined correctly
			if(z_max-z_min <= 0)
			{
				ROS_WARN("Please make sure the parameter /segment/z_min_distance is smaller than /segment/z_max_distance");
				return;
			}
		}while(z_max-z_min <= 0);

		// set parameters for z axis filtering
		pt.setInputCloud(input);
		pt.setKeepOrganized(keep_organized);
		pt.setFilterFieldName("z");
		pt.setFilterLimits(z_min, z_max);

		// reset timer to measure only the filtering
		duration(START);
		pt.filter(input_filtered);
		duration(STOP);

		// measure time for converting the message
		duration(START);
		pcl::fromROSMsg (input_filtered, *cloud);
		duration(STOP);
	}
	else
	{
		// do not measure time but also do not destroy csv
		cout <<';';
		
		// measure time for converting the message
		duration(START);
		pcl::fromROSMsg (*input, *cloud);
		duration(STOP);
	}

	// measure time to find biggest planar surface
	duration(START);
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	duration(STOP);
	
	// measure time for overwriting filtered values
	duration(START);
	extract.setIndices (inliers);
	extract.filterDirectly (cloud);
	duration(STOP);

	// write pcd file if verbose = true & measure time
	if (output_pcd)
	{
		duration(START);
		writer.write<pcl::PointXYZ> ("plane_removed.pcd", *cloud, false);
		duration(STOP);
	}
	else
		cout <<';';

	// output number of points found on a surface
	cout << inliers->indices.size() << endl;

}

