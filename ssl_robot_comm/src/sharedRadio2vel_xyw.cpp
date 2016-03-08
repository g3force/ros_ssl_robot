/**
 * Translate ssl_robot_msg::vel_xyw to SSL shared radio protobuf command
 */

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
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

void PrintRadioCommand(const RadioProtocolCommand& cmd) {
  printf("Robot %X : vx=%6.3f vy=%6.3f vr=%6.3f",
         cmd.robot_id(),
         cmd.velocity_x(),
         cmd.velocity_y(),
         cmd.velocity_r());
  if (cmd.has_flat_kick()) {
    printf(" flat_kick=%6.3f", cmd.flat_kick());
  } else if (cmd.has_chip_kick()) {
    printf(" chip_kick=%6.3f", cmd.chip_kick());
  }
  if (cmd.has_dribbler_spin()) {
    printf(" dribbler=%6.3f", cmd.dribbler_spin());
  }
  printf("\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssl_robot_sharedRadio_server");

  std::string recv_node = "/ssl_robot";
  recv_node = "/ssl_robot_affw";

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Vector3>(recv_node + "/set_vel_xyw", 1);
  
  int port_number = 10010;
  int socket_fd = 0;
  int robot_id = 0;
  
  ros::param::param<int>("robot_id", robot_id, 0);
  
  socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in server_address;
  bzero(&server_address, sizeof(server_address));
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address.sin_port = htons(port_number);
  bind(socket_fd,
       reinterpret_cast<sockaddr*>(&server_address),
       sizeof(server_address));
  sockaddr_in client_address;
  bzero(&client_address, sizeof(client_address));

  RadioProtocolWrapper radio_message;
  while (ros::ok()) {
    char buffer[65536];
    socklen_t len = sizeof(client_address);
    const int num_bytes = recvfrom(
        socket_fd,
        buffer,
        sizeof(buffer),
        0,
        reinterpret_cast<sockaddr*>(&client_address),
        reinterpret_cast<socklen_t*>(&len));
    const std::string client_name =
        inet_ntoa(client_address.sin_addr);
    if (radio_message.ParseFromArray(buffer, num_bytes)) {
      const int num_robots = radio_message.command_size();
      //~ printf("%d robot command(s) received from %s\n",
             //~ num_robots,
             //~ client_name.c_str());
      for (int i = 0; i < num_robots; ++i) {
        //~ PrintRadioCommand(radio_message.command(i));
        
        geometry_msgs::Vector3 vec;
        vec.x = radio_message.command(i).velocity_x();
        vec.y = radio_message.command(i).velocity_y();
        vec.z = radio_message.command(i).velocity_r();
        pub.publish(vec);
        ros::spinOnce();
        
      }
      //~ printf("\n");
    } else {
      printf("Error parsing protobuf received from %s\n",
             client_name.c_str());
    }
  }

  ros::shutdown();

  return 0;
}
