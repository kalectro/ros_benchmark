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

#include <segmentation.h>

using namespace std;

static const string PCD_PATH = "src/ros_benchmark/pcl_segmentation/pcds/benchmark.pcd";

void segment_cb ( const sensor_msgs::PointCloud2ConstPtr& input );

int main (int argc, char** argv)
{
	ros::init (argc, argv, "benchmark_segmentation");
	ros::NodeHandle nh("~");
	// Create a ROS subscriber for the input point cloud

	// Create publisher to publish found coefficients
	pub_coeffs = nh.advertise<pcl::ModelCoefficients> ("coefficients_cylinder", 1);

	// Create publihser to publish points mapped to cylinder
	pub_cylinder = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_cylinder", 1);

	// Write description for csv file	
	cout << "All time values measured in milli seconds" << endl;
	cout << "downsample,filter z,segment plane,compute normals,segment cylinder,write output,points total,points without plane,points on cylinder" << endl;
	
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
	nh.param("output_pcd", output_pcd, false);
	
	// get pcd path from parameter server
	string pcd_path;
	if (nh.getParam("pcd_path", pcd_path))
	{
		ROS_DEBUG("Found pcd path %s on server", pcd_path.c_str());
	}
	else // parameter for pcd path does not exist
	{
		pcd_path = PCD_PATH;
		ROS_DEBUG("Found no pcd path on server, trying %s", pcd_path.c_str());
	}
	
	// Create a pointcloud to load the pcd file to
	sensor_msgs::PointCloud2::Ptr input (new sensor_msgs::PointCloud2);
	
	// load the pcd into the point cloud
  if (pcl::io::loadPCDFile (pcd_path, *input) == -1) 
  {
    PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
    return (-1);
  }

	while( ros::ok() )
	{
	 	// get all parameters from parameter server
	  nh.param("cylinder/radius_max", radius_max, 0.1);
	  nh.param("cylinder/radius_min", radius_min, 0.0);
	  nh.param("cylinder/threshold", threshold_cylinder, 0.04);
	  nh.param("cylinder/normal_distance_weight", normal_distance_weight_cylinder, 0.1);
	  nh.param("plane/threshold", threshold_plane, 0.02);
	  nh.param("z_min_distance", z_min, 0.5);
	  nh.param("z_max_distance", z_max, 1.5);
	  nh.param("keep_organized", keep_organized, false);
	  nh.param("voxel_size", voxel_size, 0.01);
	
    segment_cb( input );
    
	  ros::spinOnce ();
  }
}

void duration(bool identifier)
{
	// 0 = STOP ; 1= START
	// start timer if START
	// stop and print timer if STOP
	if(identifier == START)
		start = ros::Time::now();
	else
	{
		stop = ros::Time::now();
		cout << (stop-start).toNSec()/1000000 << ',' ;
	}
}	

void segment_cb ( const sensor_msgs::PointCloud2ConstPtr& input )
{
	// Construct point cloud to work with
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Construct point cloud after plane removal
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane (new pcl::PointCloud<pcl::PointXYZ>);

	// Construct point cloud of found cylinder
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);

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

	// Create the cloud normals needed for cylinder segmentation
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Create KdTree needed for normal estimation
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	// Create a pointcloud to store the downsampled point cloud
	sensor_msgs::PointCloud2::Ptr input_voxeled (new sensor_msgs::PointCloud2);


	//
	// measure time for downscaling the points
	//
	// Create the filtering object and downsample the dataset using the parameter leaf size
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (voxel_size,voxel_size,voxel_size);

	// reset timer to measure only the filtering
	duration(START);
	sor.filter (*input_voxeled);
	duration(STOP);


	// 
	// measure time for filtering the points
	// 
	// check if z values are supposed to be filtered (default: true)
	if(z_max-z_min <= 0)
	{
		ROS_WARN("Please make sure the parameter z_min_distance is smaller than z_max_distance");
		return;
	}
	// set parameters for z axis filtering
	pt.setInputCloud(input_voxeled);
	pt.setKeepOrganized(keep_organized);
	pt.setFilterFieldName("z");
	pt.setFilterLimits(z_min, z_max);

	// reset timer to measure only the filtering
	duration(START);
	pt.filter(input_filtered);
	duration(STOP);

	// convert the message
	pcl::fromROSMsg (input_filtered, *cloud);


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
	// remove plane from point cloud
	// 
	extract_planes.setInputCloud(cloud);
	extract_planes.setIndices (inliers_plane);
	extract_planes.setKeepOrganized(keep_organized);
	// limit size for radius in meters
	seg_cylinder.setRadiusLimits (radius_min, radius_max);
	extract_planes.filter (*cloud_no_plane);


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
	// maximal distance from point to cylinder surface to be identified as cylinder
	seg_cylinder.setDistanceThreshold (threshold_cylinder);
	seg_cylinder.setNormalDistanceWeight (normal_distance_weight_cylinder);
	duration(START);
	seg_cylinder.segment (*inliers_cylinder, *coefficients_cylinder);
	duration(STOP);


	// 
	// overwrite filtered values
	// 
	extract_cylinders.setInputCloud (cloud_no_plane);
	extract_cylinders.setIndices (inliers_cylinder);
	extract_cylinders.setKeepOrganized(keep_organized);
	extract_cylinders.filter (*cloud_cylinder);

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
		cout <<',';

	//
	// convert back to ROS message
	//
	pcl::toROSMsg(*cloud_cylinder, output);

	// output number of points in point clouds
	cout << cloud->size() << ',' ;
	cout << cloud_no_plane->size() << ',' ;
	cout << cloud_cylinder->size() << endl;

	// fill in header
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "pointcloud_frame";

	// publish points of cylinder and its coefficients
	pub_coeffs.publish(coefficients_cylinder);
	pub_cylinder.publish(output);
}

