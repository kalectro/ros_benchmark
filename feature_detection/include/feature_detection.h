/*
 * feature_detection.h
 *
 *  Created on: Jan 15, 2013
 *      Author: frk1pal
 * This program is a very simple feature detection for benchmarking purpose
 */

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Parameters used if not available on parameter server
static const std::string IMAGE_PATH = "src/ros_benchmark/feature_detection/images/VGA1.jpg";

// stores detector type to find features in image like FAST, SIFT, SURF
std::string detector_type;

// stores extractor type to use for feature extraction like ORB, SIFT, SURF
std::string extractor_type;

// stores image path to load the image from
std::string image_path;

// set to true to print first line for csv which labels the columns
bool verbosity;

// stores number of repetition for feature detection and extraction
int repetitions;

// initialize OpenCV image, descriptor and keypoints
cv::Mat cv_image;
cv::Mat descriptors;
std::vector<cv::KeyPoint> keypoints;
