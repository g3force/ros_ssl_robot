/*
 * joy2vel_xyw.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

ros::Publisher pub_vel;
geometry_msgs::Twist curVel;
geometry_msgs::Twist targetVel;

double vDef = 1;
double vMax = 5;
double dccDef = 3;
double dccMax = 6;
double accDef = 3;
double accMax = 6;
double rotateDef = 10;
double rotateMax = 20;
double rotateMin = 5;

double uDcc = 0;
double uAcc = 0;

void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy) {

	uAcc = -(joy->axes[5]-1)/2;
	uDcc = -(joy->axes[2]-1)/2;

	double vmax = vDef + (vMax-vDef) * uAcc;
	double rotmax = rotateDef + (rotateMax-rotateDef) * uAcc;
	geometry_msgs::Twist vel;
	vel.linear.x = joy->axes[1] * vmax;
	vel.linear.y = joy->axes[0] * vmax;
	vel.angular.z = joy->axes[3] * rotmax;
	targetVel = vel;
}

double angleBetweenVectorAndVector(double v1x, double v1y, double v2x, double v2y)
{
	// angle between positive x-axis and first vector
	double angleA = atan2(v1x, v1y);
	// angle between positive x-axis and second vector
	double angleB = atan2(v2x, v2y);
	// rotation
	double rotation = angleB - angleA;
	// fix overflows
	if (rotation < (-M_PI - 0.001))
	{
		rotation += 2 * M_PI;
	} else if (rotation > (M_PI + 0.001))
	{
		rotation -= 2 * M_PI;
	}
	return fabsf(rotation);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_joy");

	ros::NodeHandle n;
	pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Subscriber sub_joy = n.subscribe("/joy", 1,
			callbackJoy);

	double dt = 0.01;
	while(ros::ok())
	{
		double acc;
		if (targetVel.linear.x == 0 && targetVel.linear.y == 0)
		{
			// dcc
			acc = (dccDef + ((dccMax - dccDef) * uDcc));
		} else if ((curVel.linear.x == 0 && curVel.linear.y == 0)
				|| (angleBetweenVectorAndVector(targetVel.linear.x, targetVel.linear.y, curVel.linear.x, curVel.linear.y) < M_PI/2))
		{
			// acc
			acc = (accDef + ((accMax - accDef) * uAcc));
		} else
		{
			// dcc
			acc = (dccDef + ((dccMax - dccDef) * uDcc));
		}

		double dx = targetVel.linear.x - curVel.linear.x;
		double dy = targetVel.linear.y - curVel.linear.y;
		double ldxy = sqrt(dx*dx+dy*dy);
		if (ldxy > (acc * dt))
		{
			curVel.linear.x = curVel.linear.x + (dx / ldxy) * (acc * dt);
			curVel.linear.y = curVel.linear.y + (dy / ldxy) * (acc * dt);
		} else
		{
			curVel.linear.x = targetVel.linear.x;
			curVel.linear.y = targetVel.linear.y;
		}

		curVel.angular.z = targetVel.angular.z;

		pub_vel.publish(curVel);
		ros::spinOnce();

		ros::Duration(dt).sleep();
	}

	return 0;
}


