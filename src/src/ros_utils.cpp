#include "ros_utils.hpp"

ros::ServiceClient LinkAtt;
ros::ServiceClient LinkDet;

void init_servclients(ros::NodeHandle &node) {
  LinkAtt = node.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  LinkDet = node.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
}

bool ros_link(ros::ServiceClient node, std::string model1, std::string link1, std::string model2, std::string link2){
  gazebo_ros_link_attacher::Attach srv;
  srv.request.model_name_1 = model1;
  srv.request.link_name_1 = link1;
  srv.request.model_name_2 = model2;
  srv.request.link_name_2 = link2;

  return node.call(srv);
}