/*
 * export_data.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include <boost/filesystem/operations.hpp>
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
#include <ros/param.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

std::ofstream fRawPose, fRawTwist, fFiltered, fSet, fTarget;

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
			<< twist->header.stamp << " " << twist->twist.linear.x << " "
			<< twist->twist.linear.y << " " << twist->twist.angular.z
			<< std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_export");
	ros::NodeHandle n;

	std::string folder = "/tmp/ssl_robot_data";
	ros::param::get("dataFolder", folder);

	std::string rawPoseFile = folder + "/raw_pose.csv";
	std::string rawTwistFile = folder + "/raw_twist.csv";
	std::string filteredFile = folder + "/filtered.csv";
	std::string setpointFile = folder + "/set.csv";
	std::string targetFile = folder + "/target.csv";

	// create folder if it does not exist
	boost::filesystem::create_directories(folder);
	// remove old files if they exist
	boost::filesystem::remove(rawPoseFile);
	boost::filesystem::remove(rawTwistFile);
	boost::filesystem::remove(filteredFile);
	boost::filesystem::remove(setpointFile);
	boost::filesystem::remove(targetFile);

	// wait for valid time
	ros::Time time;
	do {
		time = ros::Time::now();
	} while (time.isZero());

	fRawPose.open(rawPoseFile.c_str());
	fRawTwist.open(rawTwistFile.c_str());
	fFiltered.open(filteredFile.c_str());
	fSet.open(setpointFile.c_str());
	fTarget.open(targetFile.c_str());

	ros::Subscriber sub_filtered = n.subscribe("/ssl_robot/state", 1,
			callbackFiltered);
	ros::Subscriber sub_raw_pose = n.subscribe("/ssl_robot/raw_pose", 1,
			callbackRawPose);
	ros::Subscriber sub_raw_twist = n.subscribe("/ssl_robot/raw_twist", 1,
			callbackRawTwist);
	ros::Subscriber sub_set = n.subscribe("/cmd_vel", 1, callbackSet);
	ros::Subscriber sub_target = n.subscribe("/affw_ctrl/target_vel", 1,
			callbackTarget);

	ros::spin();

	fRawPose.close();
	fRawTwist.close();
	fFiltered.close();
	fSet.close();
	fTarget.close();

	return 0;
}

