/*
 * ssl_robot_transform.cpp
 *
 *  Created on: May 7, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

tf::TransformBroadcaster* odom_broadcaster;
ros::Publisher pub_odom;
nav_msgs::Odometry lastOdom;
bool first = true;


double getOrientation(geometry_msgs::Quaternion gq) {
	double ow = gq.w;
	double ox = gq.x;
	double oy = gq.y;
	double oz = gq.z;

	tf::Quaternion q(ox, oy, oz, ow);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

double normalizeAngle(double angle)
{
	// Don't call this a hack! It's numeric!
	return (angle - (round((angle / (M_PI*2)) - 1e-8) * M_PI*2));
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odo) {

	if(odo->header.frame_id != "map")
	{
		ROS_ERROR("Invalid frame id");
		return;
	}

	if(first)
	{
		first = false;
		lastOdom = *odo;
	}

	double angle = -(getOrientation(lastOdom.pose.pose.orientation));
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = odo->header.stamp;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "map";

	odom_trans.transform.translation.x = -(odo->pose.pose.position.x * cos(angle) - odo->pose.pose.position.y * sin(angle));
	odom_trans.transform.translation.y = -(odo->pose.pose.position.x * sin(angle) + odo->pose.pose.position.y * cos(angle));
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster->sendTransform(odom_trans);

	geometry_msgs::TransformStamped base_trans;
	base_trans.header.stamp = odo->header.stamp;
	base_trans.header.frame_id = "base_link";
	base_trans.child_frame_id = "odom";
	base_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
	odom_broadcaster->sendTransform(base_trans);

	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.header.stamp = odo->header.stamp;
	odom.child_frame_id = "base_link";
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	odom.twist.twist.linear.x = odo->twist.twist.linear.x * cos(angle) - odo->twist.twist.linear.y * sin(angle);
	odom.twist.twist.linear.y = odo->twist.twist.linear.x * sin(angle)	+ odo->twist.twist.linear.y * cos(angle);
	odom.twist.twist.angular.z = odo->twist.twist.angular.z;

	pub_odom.publish(odom);

	lastOdom = *odo;

	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_transform");

	ros::NodeHandle n;

	std::string odom_topic = "odom";
	ros::param::get("odom_topic", odom_topic);

	odom_broadcaster = new tf::TransformBroadcaster;
	ros::TransportHints th;
	th.unreliable();
	ros::Subscriber sub_odom = n.subscribe("state", 1,	callbackOdom, th);
	pub_odom = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

	ros::spin();

	delete odom_broadcaster;

	return 0;
}



