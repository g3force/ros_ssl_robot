/*
 * ssl-robot-affw.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <affw_ctrl/ActionRequest.h>
#include <affw_ctrl/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <vector>

ros::Publisher pub_set_vel;
ros::Publisher pub_fdbk_state;
ros::ServiceClient srv_action;

void setVelCallback(const geometry_msgs::TwistStamped::ConstPtr& vel) {

	affw_ctrl::ActionRequest srv;
	srv.request.header = vel->header;
	srv.request.setPoint.push_back(vel->twist.linear.x);
	srv.request.setPoint.push_back(vel->twist.linear.y);
	srv.request.setPoint.push_back(vel->twist.angular.z);

	if (srv_action.call(srv)) {
		geometry_msgs::Twist outVel;
		outVel.linear.x = srv.response.outVel[0];
		outVel.linear.y = srv.response.outVel[1];
		outVel.angular.z = srv.response.outVel[2];

		pub_set_vel.publish(outVel);
		ros::spinOnce();
	} else {
		ROS_ERROR("Failed to get action from affw");
	}
}

void feedbackVelCallback(const nav_msgs::Odometry::ConstPtr& vel) {
	affw_ctrl::State state;
	state.header = vel->header;
	state.vel.push_back(vel->twist.twist.linear.x);
	state.vel.push_back(vel->twist.twist.linear.y);
	state.vel.push_back(vel->twist.twist.angular.z);

	pub_fdbk_state.publish(state);
	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_affw");
	ros::NodeHandle n;

	// send velocity to robot
	pub_set_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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
