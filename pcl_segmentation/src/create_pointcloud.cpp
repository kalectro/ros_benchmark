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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

using namespace std;

string pcd_path, pcd_path_temp;

//Initialize Publisher
ros::Publisher pub;

// Parameters used if not available on parameter server
static const string PCD_PATH = "src/ros_benchmark/pcl_segmentation/pcds/benchmark.pcd";

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "create_pointcloud");
	ros::NodeHandle nh;

	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);

	// Create a pointcloud to load the pcd file to
	sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);	

	// Set publishing frequency
	ros::Rate loop_rate(1);

	while (ros::ok())
	{
		// get pcd path from parameter server
		if (nh.getParam("create_pointcloud/pcd_path", pcd_path_temp))
		{
			ROS_DEBUG("Found pcd path %s on server", pcd_path_temp.c_str());
		}
		else // parameter for pcd path does not exist
		{
			pcd_path_temp = PCD_PATH;
			ROS_DEBUG("Found no pcd path on server, trying %s", pcd_path.c_str());
		}

		// read pcd only if changed
		if (pcd_path_temp != pcd_path)
		{
			pcd_path = pcd_path_temp;
			// load the pcd into the point cloud
			if (pcl::io::loadPCDFile (pcd_path, *cloud) == -1) 
			{
				PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
				return (-1);
			}
		}

		// Fill in the header
    cloud->header.stamp = ros::Time::now();
    cloud->header.frame_id = "pointcloud_frame";

    // Publish the model coefficients
    pub.publish (*cloud);

    // Spin
    ros::spinOnce ();
    loop_rate.sleep();
	}
}
