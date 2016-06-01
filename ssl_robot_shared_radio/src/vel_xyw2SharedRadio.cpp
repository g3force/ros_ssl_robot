/**
 * Translate ssl_robot_msg::vel_xyw to SSL shared radio protobuf command
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include <string>
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

#include "radio_protocol_command.pb.h"
#include "radio_protocol_wrapper.pb.h"


int socket_fd = 0;
int robot_id = 0;
sockaddr_in server_address;

void ClearCommandProtbuf(RadioProtocolCommand* cmd_ptr) {
  RadioProtocolCommand& cmd = *cmd_ptr;
  cmd.set_robot_id(0);
  cmd.set_velocity_x(0);
  cmd.set_velocity_y(0);
  cmd.set_velocity_r(0);
}

void FillCommandProtobuf(uint32_t robot_id,
                         float velocity_x,
                         float velocity_y,
                         float velocity_r,
                         RadioProtocolCommand* cmd_ptr) {
  RadioProtocolCommand& cmd = *cmd_ptr;
  cmd.set_robot_id(robot_id);
  cmd.set_velocity_x(velocity_x);
  cmd.set_velocity_y(velocity_y);
  cmd.set_velocity_r(velocity_r);
}

void senderCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  RadioProtocolWrapper wrapper;
  RadioProtocolCommand command;
  std::string buffer;

  FillCommandProtobuf(robot_id, vel->linear.x, vel->linear.y, vel->angular.z, &command);
  *wrapper.add_command() = command;
  wrapper.SerializeToString(&buffer);
  sendto(socket_fd,
         buffer.data(),
         buffer.length(),
         0,
         reinterpret_cast<sockaddr*>(&server_address),
         sizeof(server_address));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssl_robot_sharedRadio_client");

	ros::param::param<int>("robot_id", robot_id, 0);
	std::string server_name = "192.168.20.210";
	ros::param::param<std::string>("server_name", server_name, "192.168.20.210");
	int port_number = 10010;

	socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	bzero(&server_address, sizeof(server_address));
	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = inet_addr(server_name.c_str());
	server_address.sin_port = htons(port_number);

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/cmd_vel", 1, senderCallback);

	ros::spin();

//	RadioProtocolWrapper wrapper;
//	RadioProtocolCommand command;
//	std::string buffer;

//	FillCommandProtobuf(robot_id, 0, 0, 0, &command);
//	*wrapper.add_command() = command;
//	wrapper.SerializeToString(&buffer);
//	sendto(socket_fd,
//	   buffer.data(),
//	   buffer.length(),
//	   0,
//	   reinterpret_cast<sockaddr*>(&server_address),
//	   sizeof(server_address));

	return 0;
}
