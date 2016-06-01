/*
 * test_delay.cpp
 *
 *  Created on: May 31, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */


#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "nav_msgs/Odometry.h"

#include <boost/circular_buffer.hpp>
#include <fstream>

#include <signal.h>


boost::circular_buffer<double> *delays = NULL;
int framesSeen = 0;

void mySigintHandler(int sig)
{
	double sum = 0;
	double max = 0;
	boost::circular_buffer<double> delaysCpy = *delays;
	std::ofstream fDim("/tmp/delay");
	for(boost::circular_buffer<double>::iterator it = delaysCpy.begin(); it != delaysCpy.end(); it++)
	{
		sum += *it;
		if(*it > max)
			max = *it;
		fDim << *it << std::endl;
	}
	double avg = sum/delaysCpy.size();
	std::cout << "avg delay: " << avg << std::endl;
	std::cout << "max delay: " << max << std::endl;

	fDim.close();

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

void callback(nav_msgs::Odometry::ConstPtr odo)
{
	ros::Duration diff = ros::Time::now() - odo->header.stamp;
	delays->push_back(diff.toSec());
//	std::cout << diff.toSec() << std::endl;
	framesSeen++;

	if(framesSeen >= delays->capacity() + 50)
	{
		mySigintHandler(0);
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "test_delay");

	int nFrames = 1000;
	if(argc > 1)
	{
		nFrames = atoi(argv[1]);
	}

	ros::NodeHandle n;
	ros::TransportHints th;
	th.unreliable();
	ros::Subscriber sub = n.subscribe("odom", 1, callback, th);

	delays = new boost::circular_buffer<double>(nFrames);
	signal(SIGINT, mySigintHandler);
	ros::spin();

	return 0;
}


