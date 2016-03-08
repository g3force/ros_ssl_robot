/*
 * ssl-robot-affw.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <iostream>

#include "ros/ros.h"
#include "ros/time.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include "ssl_robot_msgs/Target.h"
#include "ssl_robot_msgs/State.h"

#include "geometry_msgs/Vector3.h"
#include "affw_ctrl/State.h"
#include "affw_ctrl/ActionRequest.h"

ros::Publisher pub_set_vel;
ros::Publisher pub_fdbk_state;
ros::ServiceClient srv_action;

void setVelCallback(const ssl_robot_msgs::Target::ConstPtr& vel) {

	affw_ctrl::ActionRequest srv;
	srv.request.t = vel->t;
	srv.request.setPoint.push_back(vel->vel_x);
	srv.request.setPoint.push_back(vel->vel_y);
	srv.request.setPoint.push_back(vel->vel_w);

	if (srv_action.call(srv)) {
		geometry_msgs::Vector3 outVel;
		outVel.x = srv.response.outVel[0];
		outVel.y = srv.response.outVel[1];
		outVel.z = srv.response.outVel[2];

		pub_set_vel.publish(outVel);
		ros::spinOnce();
	} else {
		ROS_ERROR("Failed to get action from affw");
	}
}

void feedbackVelCallback(const ssl_robot_msgs::State::ConstPtr& vel) {
	affw_ctrl::State state;
	state.t = vel->t;
	state.local_vel.push_back(vel->local_vel_x);
	state.local_vel.push_back(vel->local_vel_y);
	state.local_vel.push_back(vel->local_vel_w);
	state.global_vel.push_back(vel->global_vel_x);
	state.global_vel.push_back(vel->global_vel_y);
	state.global_vel.push_back(vel->global_vel_w);
	state.global_pos.push_back(vel->global_pos_x);
	state.global_pos.push_back(vel->global_pos_y);
	state.global_pos.push_back(vel->global_pos_w);

	pub_fdbk_state.publish(state);
	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_affw");
	ros::NodeHandle n;

	// send velocity to robot
	pub_set_vel = n.advertise<geometry_msgs::Vector3>("/ssl_robot/set_vel_xyw",
			1);
	// send robot state to affw
	pub_fdbk_state = n.advertise<affw_ctrl::State>("/affw_ctrl/state", 1);

	// receive velocity cmd from external source
	ros::Subscriber sub_set_vel = n.subscribe("/ssl_robot_affw/target_vel", 1,
			setVelCallback);
	// receive robot state from robot
	ros::Subscriber sub_fdbk_vel = n.subscribe("/ssl_robot/state", 1,
			feedbackVelCallback);

	srv_action = n.serviceClient<affw_ctrl::ActionRequest>("/affw_ctrl/action");

	ros::spin();

	return 0;
}
