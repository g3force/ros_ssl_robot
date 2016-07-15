/*
 * pose_add_cov.cpp
 *
 *  Created on: 15.07.2016
 *      Author: NicolaiO
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Publisher pub_pose;

void callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	geometry_msgs::PoseWithCovarianceStamped poseCov;
	poseCov.header = pose->header;
	poseCov.pose.pose = pose->pose;

	poseCov.pose.covariance.elems[0*6 + 0] = 0.02;
	poseCov.pose.covariance.elems[1*6 + 1] = 0.02;
	poseCov.pose.covariance.elems[5*6 + 5] = 0.05;

	// send new transform
//	static tf::TransformBroadcaster br;
//	tf::Transform transform;
//	transform.setOrigin(tf::Vector3(0, 0, 0.0));
//	tf::Quaternion q;
//	q.setRPY(0,0,0);
//	transform.setRotation(q);
//	ros::Time time = pose->header.stamp - ros::Duration(0.1);
//	br.sendTransform(
//			tf::StampedTransform(transform, time, "base_link",
//					"odom"));

	pub_pose.publish(poseCov);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_add_cov");
	ros::NodeHandle n;

	std::string fromTopic = "";
	std::string toTopic = "";
	ros::param::get("from_topic", fromTopic);
	ros::param::get("to_topic", toTopic);

	ros::Time time;
	do {
		time = ros::Time::now();
	} while (time.isZero());

	if(fromTopic.empty() || toTopic.empty())
	{
		ROS_ERROR("Topics not specified.");
		ros::Duration(0.3).sleep();
		return 1;
	}

	ros::Subscriber sub_model_states = n.subscribe(fromTopic, 1, callback);
	pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(toTopic, 1);

	ros::spin();

	return 0;
}
