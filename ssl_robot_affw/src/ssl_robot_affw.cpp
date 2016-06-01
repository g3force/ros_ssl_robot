/*
 * ssl-robot-affw.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <affw_msgs/ActionRequest.h>
#include <affw_msgs/State.h>
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
#include <std_msgs/Header.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>

ros::Publisher pub_set_vel;
ros::Publisher pub_fdbk_state;
ros::ServiceClient srv_action;
ros::Time lastSetVelTime;

double normalizeAngle(double angle)
{
	// Don't call this a hack! It's numeric!
	return (angle - (round((angle / (M_PI*2)) - 1e-8) * M_PI*2));
}

double angleOfVector(double x, double y)
{
	double angle = normalizeAngle( acos(x / (sqrt( x*x + y*y ))) );
	if(x<0) angle = -angle;
	return angle;
}

void setVelCallback(const geometry_msgs::TwistStamped::ConstPtr& vel) {

	lastSetVelTime = ros::Time::now();

	affw_msgs::ActionRequest srv;
	srv.request.state.header = vel->header;
//	srv.request.setPoint.push_back(vel->twist.linear.x);
//	srv.request.setPoint.push_back(vel->twist.linear.y);

	double x = vel->twist.linear.x;
	double y = vel->twist.linear.y;
	double len = sqrt(x*x+y*y);
	srv.request.state.vel.push_back(len);

	srv.request.state.vel.push_back(vel->twist.angular.z);


	if (srv_action.call(srv)) {
		geometry_msgs::Twist outVel;
//		outVel.linear.x = srv.response.outVel[0];
//		outVel.linear.y = srv.response.outVel[1];

		outVel.linear.x = x/len * srv.response.outVel[0];
		outVel.linear.y = y/len * srv.response.outVel[0];

		outVel.angular.z = srv.response.outVel[1];

		pub_set_vel.publish(outVel);
		ros::spinOnce();
	} else {
		ROS_ERROR_THROTTLE(1, "Failed to get action from affw");
	}
}

void feedbackVelCallback(const nav_msgs::Odometry::ConstPtr& odom) {

	if(odom->header.frame_id != "odom")
	{
		ROS_ERROR("invalid frame_id");
	}

	affw_msgs::State state;
	state.header = odom->header;
	state.header.frame_id = "base_link";

//	state.vel.push_back(odom->twist.twist.linear.x);
//	state.vel.push_back(odom->twist.twist.linear.y);

	double x = odom->twist.twist.linear.x;
	double y = odom->twist.twist.linear.y;
	double len = sqrt(x*x+y*y);
	state.vel.push_back(len);

	state.vel.push_back(odom->twist.twist.angular.z);

	pub_fdbk_state.publish(state);
	ros::spinOnce();
}

void timerCallback(const ros::TimerEvent&)
{
	ros::Duration diff = ros::Time::now() - lastSetVelTime;
	if(diff.toSec() > 0.2 && diff.toSec() < 0.4)
	{
		geometry_msgs::Twist outVel;
		pub_set_vel.publish(outVel);
		ros::spinOnce();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_affw");
	ros::NodeHandle n;

	lastSetVelTime = ros::Time::now();

	// unreliable transport
	ros::TransportHints th;
	th.unreliable();

	// send velocity to robot
	pub_set_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	// send robot state to affw
	pub_fdbk_state = n.advertise<affw_msgs::State>("/affw_ctrl/state", 1);

	// receive velocity cmd from external source
	ros::Subscriber sub_set_vel = n.subscribe("/affw_ctrl/target_vel", 1,
			setVelCallback, th);

	// receive robot state from robot
	ros::Subscriber sub_fdbk_vel = n.subscribe("/odom", 1,
			feedbackVelCallback, th);

	srv_action = n.serviceClient<affw_msgs::ActionRequest>("/affw_ctrl/action");

	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

	ros::spin();

	return 0;
}
