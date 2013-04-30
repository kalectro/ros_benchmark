/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Feb 12, 2013
 *      Author: Kai Franke
 */

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
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

// define enum for duration function
enum { STOP, START };

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

int main(int, char**);
inline void duration(bool identifier);

// if true, node will write pcd file with largest plane removed to disk
bool output_pcd;

// if true filtered points will be removed from the point cloud, if false overwritten by NaN
bool keep_organized;

// declare timing variables
ros::Time start, stop;

// Initialize Subscriber
ros::Subscriber sub;

// Initialize Publisher for cylinder coefficients
ros::Publisher pub_coeffs;

// Initialize Publisher for points matched to cylinder
ros::Publisher pub_cylinder;

// Do you want to filter the depth for a specific depth range?
bool filter_z;

// Range variables for z coordinaten in pointcloud, can be changed using parameters
double z_min, z_max;

// Distance threshold for plane
double threshold_plane;

// Range for cylinder radius
double radius_min, radius_max;

// Distance threshold for cylinder
double threshold_cylinder;

// Normal distance weight for the cylinder
double normal_distance_weight_cylinder;

// Size of the downsampled voxel
double voxel_size;

// Create ROS message for filtered point cloud
sensor_msgs::PointCloud2 input_filtered;

// Create ROS message for cylinder point cloud output
sensor_msgs::PointCloud2 output;

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

// Declare writer to write PCD files
pcl::PCDWriter writer;

// Callback function when subscribed to point cloud
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);


