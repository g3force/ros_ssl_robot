/*
 * traj-gen.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include "ros/ros.h"

#include "ssl_robot_msgs/Target.h"

#include <iostream>
#include <fstream>
#include <vector>
#include "affw_ctrl/ActionRequest.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_traj_gen");
	ros::NodeHandle n;
	ros::Publisher pub_set_vel = n.advertise<ssl_robot_msgs::Target>(
			"/ssl_robot_affw/target_vel", 1);

	if(argc != 2)
	{
		std::cout << "Usage: " << argv[0] << " <traj-file>" << std::endl;
		return 1;
	}

	ros::Duration(2).sleep();

	ros::ServiceClient srv_action;
	srv_action = n.serviceClient<affw_ctrl::ActionRequest>("/affw_ctrl/action");
	bool avail = srv_action.waitForExistence();

	if(!avail)
		return 1;

	const char* filename = argv[1];

	std::vector<ssl_robot_msgs::Target> setpoints;
	std::ifstream file(filename);
	while(ros::Time::now().isZero());
	ros::Time now = ros::Time::now();
	std::cout << now << std::endl;
	double t, vx, vy, vw;
	while (file >> t >> vx >> vy >> vw) {
		ssl_robot_msgs::Target vel;
		vel.t = now + ros::Duration(t);
		vel.vel_x = vx;
		vel.vel_y = vy;
		vel.vel_w = vw;
		setpoints.push_back(vel);
	}

	std::vector<ssl_robot_msgs::Target>::iterator it = setpoints.begin();
	do {
		pub_set_vel.publish(*it);
		std::cout << it->t << " " << it->vel_x << " " << it->vel_y << " " << it->vel_w << std::endl;
		ros::Time t1 = it->t;
		it++;
		if(it < setpoints.end())
		{
			ros::Time t2 = it->t;
			ros::Duration diff = t2-t1;
			double sleep = diff.sec*1e6 + diff.nsec/1e3;
			boost::this_thread::sleep_for(boost::chrono::microseconds((long) sleep));
		}
		else
		{
			break;
		}
	} while(ros::ok());

	ssl_robot_msgs::Target vel;
	vel.t = now + ros::Duration(t);
	vel.vel_x = 0;
	vel.vel_y = 0;
	vel.vel_w = 0;
	pub_set_vel.publish(vel);

	ros::shutdown();

	return 0;
}

