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
	pub = nh.advertise<pcl::ModelCoefficients> ("coefficients", 1);

	// Write description for csv file	
	cout << "All time values measured in milli seconds" << endl;
	cout << "filter_z;ROS2PC;segment_plane;update_PC;write_output;points_on_plane" << endl;
	
	// set all available variables to a default value
	nh.setParam("/segment/radius_max", 0.1);
	nh.setParam("/segment/radius_min", 0.0);
	nh.setParam("/segment/threshold_cylinder", 0.04);
	nh.setParam("/segment/threshold_plane", 0.02);
	nh.setParam("/segment/z_min_distance", 0.5);
	nh.setParam("/segment/z_max_distance", 1.5);
	nh.setParam("/segment/keep_organized", false);
	nh.setParam("/segment/filter_z", true);

	// Set up SAC parameters for plane segmentation
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	
	// limit maximum computation time
	seg_plane.setMaxIterations (1000);

	// Set up SAC parameters for cylinder segmentation
	seg_cylinder.setOptimizeCoefficients (true);
	seg_cylinder.setModelType (pcl::SACMODEL_CYLINDER);
	seg_cylinder.setMethodType (pcl::SAC_RANSAC);
	seg_cylinder.setNormalDistanceWeight (0.1);
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
	ros:: NodeHandle nh;
	nh.getParam("/segment/radius_max", radius_max);
	nh.getParam("/segment/radius_min", radius_min);
	nh.getParam("/segment/threshold_cylinder", threshold_cylinder);
	nh.getParam("/segment/threshold_plane", threshold_plane);
	nh.getParam("/segment/z_min_distance", z_min);
	nh.getParam("/segment/z_max_distance", z_max);
	nh.getParam("/segment/keep_organized", keep_organized);
	nh.getParam("/segment/filter_z", filter_z);

	// measure time for filtering the points
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

	// measure time to find biggest planar surface
	duration(START);
	// maximal distance from point to planar surface to be identified as plane (2cm???)
	seg_plane.setDistanceThreshold (threshold_plane);
	seg_plane.setInputCloud (cloud);
	seg_plane.segment (*inliers_plane, *coefficients_plane);
	duration(STOP);

	// measure time for overwriting filtered values
	duration(START);
	extract_planes.setInputCloud(cloud);
	extract_planes.setIndices (inliers_plane);
	extract_planes.setKeepOrganized(keep_organized);
	// limit size for radius in meters
	seg_cylinder.setRadiusLimits (radius_min, radius_max);
	extract_planes.filter (*cloud_no_plane);
	duration(STOP);

	// measure time for finding normals
	duration(START);
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_no_plane);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);
	duration(STOP);

	// measure time to find the cylinder
	duration(START);
	seg_cylinder.setInputCloud (cloud_no_plane);
	seg_cylinder.setInputNormals (cloud_normals);
		// maximal distance from point to cylinder surface to be identified as cylinder (1cm???)
	seg_cylinder.setDistanceThreshold (threshold_cylinder);
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

	pub.publish(coefficients_cylinder);

}

