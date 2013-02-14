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

// define enum for duration function
enum { STOP, START };

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

int main(int, char**);
inline void duration(bool identifier);

// if true, node will write pcd file with largest plane removed to disk
bool verbosity;

// declare timing variables
ros::Time start, stop;

// Initialize Subscriber
ros::Subscriber sub;

// Range variables for z coordinaten in pointcloud, can be changed using parameters
double z_min, z_max;

// Callback function when subscribed to point cloud
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);


