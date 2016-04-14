/*
 * export_data.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <affw_ctrl/TargetRequest.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>

ros::Publisher pub_state;
ros::Publisher pub_raw_state;
ros::Publisher pub_raw_vel;

std::ofstream fRawPose, fRawTwist, fFiltered, fSet, fTarget, fAffw;

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

void callbackFiltered(const nav_msgs::Odometry::ConstPtr& odo) {

	fFiltered << std::fixed << std::setw(11) << std::setprecision(6)
			<< odo->header.stamp.toSec() << " " << odo->pose.pose.position.x
			<< " " << odo->pose.pose.position.y << " "
			<< getOrientation(odo->pose.pose.orientation) << " "
			<< odo->twist.twist.linear.x << " " << odo->twist.twist.linear.y
			<< " " << odo->twist.twist.angular.z << std::endl;
}

void callbackRawPose(
		const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {

	fRawPose << std::fixed << std::setw(11) << std::setprecision(6)
			<< pose->header.stamp.toSec() << " " << pose->pose.pose.position.x
			<< " " << pose->pose.pose.position.y << " "
			<< getOrientation(pose->pose.pose.orientation) << std::endl;
}

void callbackRawTwist(
		const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist) {


//	tf::TransformListener listener;
//	tf::StampedTransform transform;
//	try {
//		listener.lookupTransform("odom", "base_link", twist->header.stamp,
//				transform);
//	} catch (tf::TransformException &ex) {
//		ROS_ERROR("%s", ex.what());
//		ros::Duration(1.0).sleep();
//	}

//	double rot = transform.getRotation()
//
//	geometry_msgs::Twist vel_msg;
//	    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
//	                                    transform.getOrigin().x());
//	    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
//	                                  pow(transform.getOrigin().y(), 2));

	fRawTwist << std::fixed << std::setw(11) << std::setprecision(6)
			<< twist->header.stamp.toSec() << " " << twist->twist.twist.linear.x
			<< " " << twist->twist.twist.linear.y << " "
			<< twist->twist.twist.angular.z << std::endl;
}

void callbackSet(const geometry_msgs::Twist::ConstPtr& twist) {

	fSet << std::fixed << std::setw(11) << std::setprecision(6)
			<< ros::Time::now().toSec() << " " << twist->linear.x << " "
			<< twist->linear.y << " " << twist->angular.z << std::endl;
}

void callbackTarget(const geometry_msgs::TwistStamped::ConstPtr& twist) {

	fTarget << std::fixed << std::setw(11) << std::setprecision(6)
			<< ros::Time::now().toSec() << " " << twist->twist.linear.x << " "
			<< twist->twist.linear.y << " " << twist->twist.angular.z << std::endl;
}

void callbackAffw(const affw_ctrl::TargetRequest::ConstPtr& tr) {

	fAffw << std::fixed << std::setw(11) << std::setprecision(6)
			<< tr->header.stamp.toSec();
	for(int i=0;i<tr->state.size();i++)
		fAffw << " " << tr->state[i];
	for(int i=0;i<tr->target.size();i++)
		fAffw << " " << tr->target[i];
	for(int i=0;i<tr->action.size();i++)
		fAffw << " " << tr->action[i];
	for(int i=0;i<tr->actionComp.size();i++)
		fAffw << " " << tr->actionComp[i];

	fAffw << std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_export_data");
	ros::NodeHandle n;

	std::string folder = "/tmp/ssl_robot_data";
	if (argc > 1)
		folder = argv[1];

	boost::filesystem::create_directories(folder);

	ros::Time time;
	do {
		time = ros::Time::now();
	} while (time.isZero());

	std::string rawPoseFile = folder + "/raw_pose.csv";
	std::string rawTwistFile = folder + "/raw_twist.csv";
	std::string filteredFile = folder + "/filtered.csv";
	std::string setpointFile = folder + "/set.csv";
	std::string targetFile = folder + "/target.csv";
	std::string affwFile = folder + "/affw.csv";
	fRawPose.open(rawPoseFile.c_str());
	fRawTwist.open(rawTwistFile.c_str());
	fFiltered.open(filteredFile.c_str());
	fSet.open(setpointFile.c_str());
	fTarget.open(targetFile.c_str());
	fAffw.open(affwFile.c_str());

	ros::Subscriber sub_filtered = n.subscribe("/ssl_robot/state", 1,
			callbackFiltered);
	ros::Subscriber sub_raw_pose = n.subscribe("/ssl_robot/raw_pose", 1,
			callbackRawPose);
	ros::Subscriber sub_raw_twist = n.subscribe("/ssl_robot/raw_twist", 1,
			callbackRawTwist);
	ros::Subscriber sub_set = n.subscribe("/cmd_vel", 1, callbackSet);
	ros::Subscriber sub_target = n.subscribe("/ssl_robot_affw/target_vel", 1, callbackTarget);
	ros::Subscriber sub_affw = n.subscribe("/affw_ctrl/target_request", 1, callbackAffw);

	ros::spin();

	fRawPose.close();
	fRawTwist.close();
	fFiltered.close();
	fSet.close();
	fTarget.close();
	fAffw.close();

	return 0;
}

