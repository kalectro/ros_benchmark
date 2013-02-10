/*
 * feature_detection.h
 *
 *  Created on: Jan 15, 2013
 *      Author: frk1pal
 * This program is a very simple feature detection for benchmarking purpose
 */

#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "algorithm"
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// Parameters used if not available on parameter server
static const string IMAGE_PATH = "src/image_benchmark/images/VGA1.jpg";
