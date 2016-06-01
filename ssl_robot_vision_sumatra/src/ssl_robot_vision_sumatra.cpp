#define PORT 42001
#define MULTICAST_IP "224.5.23.3"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "sumatra_wf_export.pb.h"

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
int seq = 0;

int robot_id;
std::string robot_team_color;

namespace wpe = edu::tigers::sumatra::wp::exporter;

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

static void onPacketReceived(char* data, size_t dataSize)
{
  wpe::WorldFrame wf;
	try {
		bool parsed = wf.ParseFromArray(data, dataSize);
		if(!parsed)
			ROS_ERROR_STREAM("Could not parse data (size: " << dataSize << ")");
        else {
            for (int i=0; i<wf.bots_size();i++)
            {
                wpe::Bot bot = wf.bots().Get(i);

                if(bot.id() == robot_id
                        && (((bot.teamcolor() == wpe::BLUE) && (robot_team_color == "B"))
                            || ((bot.teamcolor() == wpe::YELLOW) && (robot_team_color == "Y")))
                        )
                {
                	nav_msgs::Odometry odom;
                	odom.header.frame_id = "map";
                	odom.header.seq = seq++;
                	odom.header.stamp = ros::Time::now();
                	odom.pose.pose.position.x = bot.pos().x();
                	odom.pose.pose.position.y = bot.pos().y();
                	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(bot.pos().z());

                	odom.twist.twist.linear.x = bot.vel().x();
                	odom.twist.twist.linear.y = bot.vel().y();
                	odom.twist.twist.angular.z = bot.vel().z();

                    pub_state.publish(odom);
                    ros::spinOnce();
                    break;
                }
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
	ros::init(argc, argv, "sumatra_wf_receiver");

	ros::NodeHandle n;
	pub_state = n.advertise<nav_msgs::Odometry>("/state", 1);

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
