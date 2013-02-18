#include "segmentation.h"

using namespace std;

int main (int argc, char** argv)
{
	ros::init (argc, argv, "object_recognition");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("pointcloud", 1, cloud_cb);

	// Create publisher to publish found coefficients
	pub_coeffs = nh.advertise<pcl::ModelCoefficients> ("coefficients_cylinder", 1);

	// Create publihser to publish points mapped to cylinder
	pub_cylinder = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_cylinder", 1);

	// Write description for csv file	
	cout << "All time values measured in milli seconds" << endl;
	cout << "filter_z;ROS2PC;segment_plane;remove_plane;compute_normals;segment_cylinder;extract_cylinder;write_output;PC2ROS;points;points_no_plane;point_on_cylinder" << endl;
	
	// set all available variables to a default value to make them visible to the user
	nh.setParam("/ros_benchmark/pcl_segmentation/cylinder/radius_max", 0.1);
	nh.setParam("/ros_benchmark/pcl_segmentation/cylinder/radius_min", 0.0);
	nh.setParam("/ros_benchmark/pcl_segmentation/cylinder/threshold", 0.04);
	nh.setParam("/ros_benchmark/pcl_segmentation/cylinder/normal_distance/weight", 0.1);
	nh.setParam("/ros_benchmark/pcl_segmentation/plane/threshold", 0.02);
	nh.setParam("/ros_benchmark/pcl_segmentation/z_min_distance", 0.5);
	nh.setParam("/ros_benchmark/pcl_segmentation/z_max_distance", 1.5);
	nh.setParam("/ros_benchmark/pcl_segmentation/keep_organized", false);
	nh.setParam("/ros_benchmark/pcl_segmentation/filter_z", true);

	// Set up SAC parameters for plane segmentation
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000);

	// Set up SAC parameters for cylinder segmentation
	seg_cylinder.setOptimizeCoefficients (true);
	seg_cylinder.setModelType (pcl::SACMODEL_CYLINDER);
	seg_cylinder.setMethodType (pcl::SAC_RANSAC);
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
	ros:: NodeHandle nh;
	// get all parameters from parameter server
	nh.getParam("/ros_benchmark/pcl_segmentation/cylinder/radius_max", radius_max);
	nh.getParam("/ros_benchmark/pcl_segmentation/cylinder/radius_min", radius_min);
	nh.getParam("/ros_benchmark/pcl_segmentation/cylinder/threshold", threshold_cylinder);
	nh.getParam("/ros_benchmark/pcl_segmentation/cylinder/normal_distance/weight", normal_distance_weight_cylinder);
	nh.getParam("/ros_benchmark/pcl_segmentation/plane/threshold", threshold_plane);
	nh.getParam("/ros_benchmark/pcl_segmentation/z_min_distance", z_min);
	nh.getParam("/ros_benchmark/pcl_segmentation/z_max_distance", z_max);
	nh.getParam("/ros_benchmark/pcl_segmentation/keep_organized", keep_organized);
	nh.getParam("/ros_benchmark/pcl_segmentation/filter_z", filter_z);


	// 
	// measure time for filtering the points
	// 
	// check if parameters for limiting z value exists
	if (filter_z)
	{
		if(z_max-z_min <= 0)
		{
			ROS_WARN("Please make sure the parameter /segment/z_min_distance is smaller than /segment/z_max_distance");
			return;
		}

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


	// 
	// measure time to find biggest planar surface
	// 
	// set maximal distance from point to planar surface to be identified as plane
	seg_plane.setDistanceThreshold (threshold_plane);
	seg_plane.setInputCloud (cloud);
	duration(START);
	seg_plane.segment (*inliers_plane, *coefficients_plane);
	duration(STOP);


	// 
	// measure time for overwriting filtered values
	// 
	extract_planes.setInputCloud(cloud);
	extract_planes.setIndices (inliers_plane);
	extract_planes.setKeepOrganized(keep_organized);
	// limit size for radius in meters
	seg_cylinder.setRadiusLimits (radius_min, radius_max);
	duration(START);
	extract_planes.filter (*cloud_no_plane);
	duration(STOP);


	// 
	// measure time for finding normals
	// 
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_no_plane);
	ne.setKSearch (50);
	duration(START);
	ne.compute (*cloud_normals);
	duration(STOP);


	// 
	// measure time to find the cylinder
	//
	seg_cylinder.setInputCloud (cloud_no_plane);
	seg_cylinder.setInputNormals (cloud_normals);
	// maximal distance from point to cylinder surface to be identified as cylinder (1cm???)
	seg_cylinder.setDistanceThreshold (threshold_cylinder);
	seg_cylinder.setNormalDistanceWeight (normal_distance_weight_cylinder);
	duration(START);
	seg_cylinder.segment (*inliers_cylinder, *coefficients_cylinder);
	duration(STOP);


	// 
	// measure time for overwriting filtered values
	// 
	extract_cylinders.setInputCloud (cloud_no_plane);
	extract_cylinders.setIndices (inliers_cylinder);
	extract_cylinders.setKeepOrganized(keep_organized);
	duration(START);
	extract_cylinders.filter (*cloud_cylinder);
	duration(STOP);

	//
	// write pcd file if verbose = true & measure time
	//
	if (output_pcd)
	{
		duration(START);
		writer.write<pcl::PointXYZ> ("cylinder.pcd", *cloud_cylinder, false);
		duration(STOP);
	}
	else
		cout <<';';

	//
	// measure time to convert back to ROS message
	//
	duration(START);
	pcl::toROSMsg(*cloud_cylinder, output);
	duration(STOP);

	// output number of points in point clouds
	cout << (*cloud).size() << ';' ;
	cout << (*cloud_no_plane).size() << ';' ;
	cout << (*cloud_cylinder).size() << endl;

	// fill in header
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "pointcloud_frame";

	// publish points of cylinder and its coefficients
	pub_coeffs.publish(coefficients_cylinder);
	pub_cylinder.publish(output);
}

