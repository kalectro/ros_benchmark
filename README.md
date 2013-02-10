ros_benchmark
===============

This metapackage is a collection of independent benchmarks for different parts of ROS and some other libraries. It was created to evaluate the performance of embedded systems running ROS. The package contains the following benchmark tests:

rosinit - measures the time for creating a node handle, call rosinit and retrieve a parameter

image_transmission - measures the time it takes to transmit 30 images using different transmission formats

feature detection - Uses different feature detectors and descriptors and outputs the time it took for one image

pcl_segmentation - extracts planes from a pointcloud
