/**
 * Translate ssl_robot_msg::vel_xyw to motor controller commands for 
 * gazebo
 */

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"

#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265359

ros::Publisher ctrl_pub[4];
double theta[4];
double bot_radius = 0.08;
double wheel_radius = 0.025;

using namespace std;

static double degToRad(double b)
{
	return b * PI / 180.0;
}

void senderCallback(const geometry_msgs::Vector3::ConstPtr& vel)
{
  // fr, rr, rl, fl
  for(int i=0;i<4;i++)
  {
    double w = (
              -sin(theta[i]) * vel->x
             + cos(theta[i]) * vel->y
             + bot_radius * vel->z
           ) * -1.0/wheel_radius;
    std_msgs::Float64 msg;
    msg.data = w;
    ctrl_pub[i].publish(msg);
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssl_robot_setvel_listener_sim");
  
  double alpha = degToRad(60);
  double beta = degToRad(45);
  theta[0] = -alpha;
  theta[1] = alpha;
  theta[2] = PI - beta;
  theta[3] = PI + beta;

  ros::NodeHandle n;

  ctrl_pub[0] = n.advertise<std_msgs::Float64>("/ssl_robot/wheel_fr_vel_controller/command", 1);
  ctrl_pub[1] = n.advertise<std_msgs::Float64>("/ssl_robot/wheel_fl_vel_controller/command", 1);
  ctrl_pub[2] = n.advertise<std_msgs::Float64>("/ssl_robot/wheel_rl_vel_controller/command", 1);
  ctrl_pub[3] = n.advertise<std_msgs::Float64>("/ssl_robot/wheel_rr_vel_controller/command", 1);
  
  ros::Subscriber sub = n.subscribe("/ssl_robot/set_vel_xyw", 1, senderCallback);

  ros::spin();

  return 0;
}
