/*
 * AngleMath.h
 *
 *  Created on: Mar 11, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef SSL_ROBOT_SSL_ROBOT_FEEDBACK_GAZ_SRC_ANGLEMATH_H_
#define SSL_ROBOT_SSL_ROBOT_FEEDBACK_GAZ_SRC_ANGLEMATH_H_

#include <geometry_msgs/Quaternion.h>

double normalizeAngle(double angle);
double angleDiff(double angle1, double angle2);
double getOrientation(geometry_msgs::Quaternion gq);

#endif /* SSL_ROBOT_SSL_ROBOT_FEEDBACK_GAZ_SRC_ANGLEMATH_H_ */
