#include "include/EndPoint.hpp"
#include "include/robot.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
//#include "std_msgs/JointStateController.h"
#include "control_msgs/JointControllerState.h"
#include <iostream>
// sudo apt install libeigen3-dev
#include <eigen3/Eigen/Eigen>
// srv from gazebo_ros_link_attacher
#include "gazebo_ros_link_attacher/Attach.h"
#include <cmath>
#include <ostream>
#include <vector>
#include <unordered_map>
#include <queue>

#include "robot.hpp"
#include "callbacks.hpp"
#include "ur5.hpp"
#include "BrickType.hpp"
#include "EndPoint.hpp"
#include "Brick.hpp"
#include "ros_utils.hpp"

int state = 0;

ur5_gazebo::brick::ConstPtr getBrick(int state);

Eigen::Vector<double, 6> A(0, -0.425, -0.3922, 0, 0, 0);
Eigen::Vector<double, 6> D(0.089159, 0, 0, 0.10915, 0.09465, 0.0823);

std::vector<BrickType> brickTypes;
std::unordered_map<std::string, EndPoint> endpoints;
std::queue<CastlePoint> castlepoints; //da riempire
EndPoint la_terra_di_mezzo;

std::string exercode = "abc";

int main(int argc, char **argv) {
  ros::init(argc, argv, "mover");
  ros::NodeHandle node;
  ros::Rate loop_rate(RATE);

  brickTypes.push_back(BrickType(1,1,2,"X1-Y1-Z2"));              //0
  brickTypes.push_back(BrickType(1,2,1,"X1-Y2-Z1"));              //1
  brickTypes.push_back(BrickType(1,2,2,"X1-Y2-Z2"));              //2
  brickTypes.push_back(BrickType(1,2,2,"X1-Y2-Z2-CHAMFER"));      //3
  brickTypes.push_back(BrickType(1,2,2,"X1-Y2-Z2-TWINFILLET"));   //4
  brickTypes.push_back(BrickType(1,3,2,"X1-Y3-Z2"));              //5
  brickTypes.push_back(BrickType(1,3,2,"X1-Y3-Z2-FILLET"));       //6
  brickTypes.push_back(BrickType(1,4,1,"X1-Y4-Z1"));              //7
  brickTypes.push_back(BrickType(1,4,2,"X1-Y4-Z2"));              //8
  brickTypes.push_back(BrickType(2,2,2,"X2-Y2-Z2"));              //9
  brickTypes.push_back(BrickType(2,2,2,"X2-Y2-Z2-FILLET"));       //10

  endpoints[brickTypes[0].name] = EndPoint(Eigen::Vector3d(-0.54,0.432498,0.88), "end_table_right", "top_plate");
  endpoints[brickTypes[1].name] = EndPoint(Eigen::Vector3d(-0.54,0.271021,0.88), "end_table_right", "top_plate");
  endpoints[brickTypes[2].name] = EndPoint(Eigen::Vector3d(-0.54,0.116340,0.88), "end_table_right", "top_plate");
  endpoints[brickTypes[3].name] = EndPoint(Eigen::Vector3d(-0.54,-0.032723,0.88), "end_table_right", "top_plate");
  endpoints[brickTypes[4].name] = EndPoint(Eigen::Vector3d(-0.54,-0.182926,0.88), "end_table_right", "top_plate");
  endpoints[brickTypes[5].name] = EndPoint(Eigen::Vector3d(0.150188,-0.409468,0.88), "end_table_left", "top_plate");
  endpoints[brickTypes[6].name] = EndPoint(Eigen::Vector3d(-0.046790,-0.409468,0.88), "end_table_left", "top_plate");
  endpoints[brickTypes[7].name] = EndPoint(Eigen::Vector3d(-0.242314,-0.409468,0.88), "end_table_left", "top_plate");
  endpoints[brickTypes[8].name] = EndPoint(Eigen::Vector3d(-0.425943,-0.409468,0.88), "end_table_left", "top_plate");
  endpoints[brickTypes[9].name] = EndPoint(Eigen::Vector3d(-0.54,0.572324, 0.88), "end_table_right", "top_plate");
  endpoints[brickTypes[10].name] = EndPoint(Eigen::Vector3d(-0.603354,-0.409468,0.88), "end_table_left", "top_plate");
  la_terra_di_mezzo = EndPoint(Eigen::Vector3d(0.0,0.5,0.885), "", "");

  castlepoints.push(CastlePoint(Eigen::Vector3d(-0.54,0.432498,0.9), brickTypes[1], "end_table_right", "top_plate"));

  init_servclients(node);
  
  Robot arm(A, D, node);

  ros::Subscriber cisco = node.subscribe("/motion/plan", RATE, cisco_message_callback);
  ros::Subscriber got = node.subscribe("/got/blocks", RATE, gotblocks_message_callback);
  ros::Publisher exercise = node.advertise<std_msgs::String>("/spawner/blocchi", RATE);

  //ros::Subscriber states = node.subscribe("/gazebo/link_states", RATE, states_message_callback);
  
  //arm.setAngle(behind_angle);
  /* ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep(); */

  // time_t start = std::time(0);

  std::cout << "kin starting" << std::endl;

  while (!gotblocks) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "kin starting2" << std::endl;

  while(ros::ok()){
    ros::spinOnce();
    //   ros::Duration(0.5).sleep();
    if(gotblocks){  // lo sleep sopra serve per le tempistiche con lo spawner, questo if serve per lo switch autoimatico degli esercizi
      if(!bricks.empty()) { // e' aggiornato da una callback senno il prgramma esce e basta perche' non fa a tempo a vedere i blocchi nella coda
        Eigen::Vector3d pe; // cpp molto piu veloce di python
        Eigen::Vector3d rpy;
        // ur5_gazebo::brick::ConstPtr b = bricks.front();
        ur5_gazebo::brick::ConstPtr b = getBrick(state == 3 ? 1 : 0);
        // bricks.pop();
        int i = 0;
        for(auto x : b->initPosition) {
          pe(i++) = i == 2 ? x - 0.89 : -x;
        }
        i=0;
        for(auto x : b->initRotation) {
          rpy(i++) = x;
        }
        pos_rot_TypeDef brick_pos = {
          .pe = pe,
          .Re = eul2rotm(rpy)
        };
        
        Brick brick(brick_pos, &brickTypes[b->type]);

        if(!(arm.moveBlockToEndpoint(brick, /*state == 3 ? castlepoints.front()*/ endpoints[brick.type()->name]))) std::cout << "failed" << std::endl;
        // if (state == 3) castlepoints.pop(); // questo serve ma lascialo commentato che al momento non lo usi
        
    } else {                                     // questo else e' da tenere
      gotblocks = 0;
      if(state > 2) break;                         // e pubblica a gio la lettera corispettiva al esercizio successivo
      ros::spinOnce();                             // usa la variabile globale state che si trova su robot.hpp che serve anche per
      std_msgs::String tmp;                        // il nome dei blocchi
    tmp.data = exercode.at(state++);
      exercise.publish(tmp);
    }
    }
  }
  // time_t end = std::time(0);
  // std::cout << "Time of execution: " << end - start;

  std::cout << "exiting\n";

  return 0;
}

ur5_gazebo::brick::ConstPtr getBrick(int state){ //da controllare e anche da controllare la classe castlepoint
  ur5_gazebo::brick::ConstPtr tmp;
  if (state){
    while (1) {
      //if (bricks.front() == castlepoints.front()) break;
      ros::spinOnce();
      bricks.push(bricks.front());
      bricks.pop();
    }
  }  
  tmp = bricks.front();
  bricks.pop();
  return tmp;
}
