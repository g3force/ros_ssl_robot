#!/bin/bash

rostopic pub -r 100 /gazebo/set_model_state gazebo_msgs/ModelState "{model_name: ssl_robot, pose: { position: { x: $1, y: $2, z: $3 }, orientation: {x: $4, y: $5, z: $6, w: $7 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }"
#rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState "{model_name: create, pose: { position: { x: $1, y: $2, z: $3 }, orientation: {x: $4, y: $5, z: $6, w: $7 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }"
#rostopic pub -r 20 /gazebo/set_model_state gazebo_msgs/ModelState "{model_name: hector1, pose: { position: { x: $1, y: $2, z: $3 }, orientation: {x: $4, y: $5, z: $6, w: $7 } }, twist: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }"
