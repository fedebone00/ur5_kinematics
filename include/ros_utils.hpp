#pragma once

#include "ros/ros.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include <string>

extern ros::ServiceClient LinkAtt;
extern ros::ServiceClient LinkDet;

void init_servclients(ros::NodeHandle &node);

bool ros_link(ros::ServiceClient node, std::string model1, std::string link1, std::string model2, std::string link2);