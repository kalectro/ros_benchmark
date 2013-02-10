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

inline double clock2 () { //returns current time in micro seconds
    timeval tv;
    gettimeofday(&tv, NULL);
	return tv.tv_usec;
}

int main(int argc, char **argv)
{
	// start timer
	start = clock2();
	// Detect feature points
	ros::init(argc, argv, "rosinit_benchmark");
	// stop the timer
	stop = clock2();
	if(stop-start < 0)
		stop+=1000000;
	// output time for detector
	cout << stop-start << ';' ;
  	
	// start timer
	start = clock2();
	// Detect feature points
	ros::NodeHandle nh;
	// stop the timer
	stop = clock2();
	if(stop-start < 0)
		stop+=1000000;
	// output time for detector
	cout << stop-start << ';' ;

	// start timer
	start = clock2();
	// get parameters from server
	nh.getParam("/rosinit_benchmark/testparam", dump);
	// stop the timer
	stop = clock2();
	if(stop-start < 0)
		stop+=1000000;
	// output time for detector
	cout << stop-start << '\n';

	return 0;
}
