#include "segmentation.h"

using namespace std;

// Construct point cloud to work with
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// Construct point cloud after plane removal
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane (new pcl::PointCloud<pcl::PointXYZ>);

// construct coefficients for plane
pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

// construct coefficients for cylinder
pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

// constructor for point found as part of planar surface
pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

// constructor for point found as part of cylinder
pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

// Create pass through point cloud for point filtering
pcl::PassThrough<sensor_msgs::PointCloud2> pt(false);

// Create ROS message for filtered point cloud
sensor_msgs::PointCloud2 input_filtered;

// Declare the segmentation object for planes
pcl::SACSegmentation<pcl::PointXYZ> seg_plane;

// Declare the segmentation object for cylinders
pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinder;

// Declare the filtering object for planes
pcl::ExtractIndices<pcl::PointXYZ> extract_planes;

// Declare the filtering object for cylinders
pcl::ExtractIndices<pcl::PointXYZ> extract_cylinders;

// Declare variable for normal estimation
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

// Create the cloud normals needed for cylinder segmentation
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

// Create KdTree needed for normal estimation
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

// Declare writer to write PCD files
pcl::PCDWriter writer;


int main (int argc, char** argv)
{
	ros::init (argc, argv, "object_recognition");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("pointcloud", 1, cloud_cb);

	// Write description for csv file	
	cout << "All time values measured in milli seconds" << endl;
	cout << "filter_z;ROS2PC;segment_plane;update_PC;write_output;points_on_plane" << endl;
	
	// Set up SAC parameters for plane segmentation
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	// maximal distance from point to planar surface to be identified as plane (2cm???)
	seg_plane.setDistanceThreshold (0.02);
	// limit maximum computation time
	seg_plane.setMaxIterations (1000);

	// Set up SAC parameters for cylinder segmentation
	seg_cylinder.setOptimizeCoefficients (true);
	seg_cylinder.setModelType (pcl::SACMODEL_CYLINDER);
	seg_cylinder.setMethodType (pcl::SAC_RANSAC);
	seg_cylinder.setNormalDistanceWeight (0.1);
	// maximal distance from point to cylinder surface to be identified as cylinder (1cm???)
	seg_cylinder.setDistanceThreshold (0.02);
	// limit size for radius
	seg_cylinder.setRadiusLimits (0, 0.1);
	// limit maximum computation time
	seg_cylinder.setMaxIterations (1000);
	
	// Extract the found plane to remove the table
	extract_planes.setNegative (true);

	// Extract the found cylinder
	extract_cylinders.setNegative (false);

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
	seg_plane.setInputCloud (cloud);
	seg_plane.segment (*inliers_plane, *coefficients_plane);
	duration(STOP);

	// measure time for overwriting filtered values
	duration(START);
	extract_planes.setInputCloud(cloud);
	extract_planes.setIndices (inliers_plane);
	extract_planes.setKeepOrganized(keep_organized);
	extract_planes.filter (*cloud_no_plane);
	duration(STOP);

	// measure time for finding normals
	duration(START);
	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_no_plane);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);
	duration(STOP);

	// measure time to find biggest cylinder
	duration(START);
	seg_cylinder.setInputCloud (cloud_no_plane);
	seg_cylinder.setInputNormals (cloud_normals);
	seg_cylinder.segment (*inliers_cylinder, *coefficients_cylinder);
	duration(STOP);

	// measure time for overwriting filtered values
	duration(START);
	extract_cylinders.setIndices (inliers_cylinder);
	extract_cylinders.filterDirectly (cloud_no_plane);
	duration(STOP);

	// write pcd file if verbose = true & measure time
	if (output_pcd)
	{
		duration(START);
		writer.write<pcl::PointXYZ> ("plane_removed.pcd", *cloud_no_plane, false);
		duration(STOP);
	}
	else
		cout <<';';

	// output number of points found on a surface
	cout << inliers_plane->indices.size() << ';' << inliers_cylinder->indices.size() << endl;

}

