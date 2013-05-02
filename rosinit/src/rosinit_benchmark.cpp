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
  cout << stop-start << ',' ;
	
	//
  // measure time for node handle creation
  //
  // start timer
  start = getSecs() * 1000000.0 + getMicroSecs();
  ros::NodeHandle nh;
  // stop the timer
  stop = getSecs() * 1000000.0 + getMicroSecs();
  // output time for detector
  cout << stop-start << ',' ;

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
