/*
 * AngleMath.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "AngleMath.h"

#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <cmath>

double normalizeAngle(double angle) {
	// Don't call this a hack! It's numeric!
	return (angle - (round((angle / (M_PI*2)) - 1e-6) * M_PI*2));
}

double angleDiff(double angle1, double angle2) {
	return normalizeAngle(normalizeAngle(angle1) - normalizeAngle(angle2));
}

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

