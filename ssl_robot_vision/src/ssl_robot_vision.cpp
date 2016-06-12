#define PORT 10006
#define MULTICAST_IP "224.5.23.2"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"

#include <string>
#include <iostream>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <tf/transform_datatypes.h>

ros::Publisher pub_state;

boost::thread* recv_thread = NULL;
boost::asio::io_service io_service;
boost::asio::ip::udp::socket m_socket(io_service);
boost::asio::ip::udp::endpoint sender_endpoint;

enum {
  max_length = 32768
};
char data_[max_length];

int robot_id;
std::string robot_team_color;
int lastCamId = 0;
double tLastCamId = 0;

static void onPacketReceived(char* data, size_t dataSize);

static void handle_receive_from(const boost::system::error_code& error,
		size_t bytes_recvd) {
	if (!error) {
		onPacketReceived(data_, bytes_recvd);

			m_socket.async_receive_from(boost::asio::buffer(data_, max_length),
				sender_endpoint,
				boost::bind(&handle_receive_from,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
	} else {
		ROS_ERROR_STREAM(error.message());
	}
}
  
static void run()
{
	m_socket.async_receive_from(boost::asio::buffer(data_, max_length),
			sender_endpoint,
			boost::bind(&handle_receive_from,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
	io_service.run();
}

const SSL_DetectionRobot* findBot(SSL_DetectionFrame& frame)
{
	if( robot_team_color == "B")
	{
		for(int i=0;i<frame.robots_blue_size(); i++)
		{
			if(frame.robots_blue(i).robot_id() == robot_id)
			{
				return &frame.robots_blue(i);
			}
		}
	} else if( robot_team_color == "Y")
	{
		for(int i=0;i<frame.robots_yellow_size(); i++)
		{
			if(frame.robots_yellow(i).robot_id() == robot_id)
			{
				return &frame.robots_yellow(i);
			}
		}
	}
	return NULL;
}

static void onPacketReceived(char* data, size_t dataSize)
{
    SSL_WrapperPacket packet;
	try {
		bool parsed = packet.ParseFromArray(data, dataSize);
		if(!parsed)
			ROS_ERROR_STREAM("Could not parse data (size: " << dataSize << ")");
        else {
        	SSL_DetectionFrame frame = packet.detection();
        	if(frame.camera_id() != lastCamId)
        	{
        		double tDiff = frame.t_capture() - tLastCamId;
        		if(tDiff < 0.1)
        		{
        			return;
        		}
        		lastCamId = frame.camera_id();
        	}
        	const SSL_DetectionRobot* bot = findBot(frame);
        	if(bot != NULL)
        	{
        		tLastCamId = frame.t_capture();

				nav_msgs::Odometry odom;
				odom.header.frame_id = "map";
				odom.header.stamp = ros::Time(frame.t_capture());
				odom.pose.pose.position.x = bot->x() / 1000.0;
				odom.pose.pose.position.y = bot->y() / 1000.0;
				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(bot->orientation());

				pub_state.publish(odom);
				ros::spinOnce();
            }
		}
	} catch(std::exception& e)
	{
		ROS_ERROR_STREAM("Exception in onPacketReceived: " << e.what());
	}
}
void mySigintHandler(int sig)
{
	io_service.stop();
	if(recv_thread != NULL) {
		delete recv_thread;
		recv_thread = NULL;
	}

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssl_robot_vision");

	ros::NodeHandle n;
	pub_state = n.advertise<nav_msgs::Odometry>("/vision/state", 1);

	ros::param::param<int>("robot_id", robot_id, 8);
	ros::param::param<std::string>("robot_team_color", robot_team_color, "Y");

	std::string netIp = "0.0.0.0";

	// Create the socket so that multiple may be bound to the same address.
	boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::address::from_string(netIp),
		PORT);
	m_socket.open(listen_endpoint.protocol());
	m_socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
	m_socket.bind(listen_endpoint);

	// Join the multicast group.
	m_socket.set_option(
		boost::asio::ip::multicast::join_group(boost::asio::ip::address::from_string(MULTICAST_IP)));

	signal(SIGINT, mySigintHandler);

	recv_thread = new boost::thread(boost::bind(&run));
	ros::spin();

	return 0;
}
