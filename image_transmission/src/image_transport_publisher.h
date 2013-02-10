/*
 * imageSource.h
 *
 *  Created on: Jan 08, 2013
 *      Author: Kai Franke
 * One image after another is read from a file and published in a user-defined frequency.
 * The images has to be .png, .jpeg or .jpg - Files. The image and the imageInformation is published in two
 * separate messages.
 */

#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
// Parameters used if not available on parameter server
static const string IMAGE_PATH = "src/image_benchmark/images/rgb.jpg";

int main(int, char**);




