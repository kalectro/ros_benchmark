/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Jan 15, 2013
 *      Author: frk1pal
 */


#include "ros/ros.h"
#include <stdio.h>
#include <sys/time.h>

using namespace std;

// variables for timing
double start, stop;
bool dump;

// returns current time in micro seconds
inline double getMicroSecs () 
{
  timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_usec;
}

// returns current time in seconds
inline double getSecs () 
{
  timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec;
}

int main(int argc, char **argv)
{
  //
  // measure time for ROS init
  //
  // start timer
  start = getSecs() * 1000000.0 + getMicroSecs();
  ros::init(argc, argv, "rosinit_benchmark");
  // stop the timer
  stop = getSecs() * 1000000.0 + getMicroSecs();
  // output time for detector
  cout << stop-start << ';' ;
	
	//
  // measure time for node handle creation
  //
  // start timer
  start = getSecs() * 1000000.0 + getMicroSecs();
  ros::NodeHandle nh;
  // stop the timer
  stop = getSecs() * 1000000.0 + getMicroSecs();
  // output time for detector
  cout << stop-start << ';' ;

  //
  // measure time for retrieving a parameter
  //
  // start timer
  start = getSecs() * 1000000.0 + getMicroSecs();
  nh.getParam("/rosinit_benchmark/testparam", dump);
  // stop the timer
  stop = getSecs() * 1000000.0 + getMicroSecs();
  // output time for detector
  cout << stop-start << '\n';

  return 0;
}
