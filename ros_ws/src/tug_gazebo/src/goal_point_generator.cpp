#include "ros/package.h"
#include "ros/ros.h"

#include "geometry/tug_environment.hpp"
#include <master/Waypoint.h>
#include <string>

Tug::Environment environment_;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "goal_points_generator");
  ros::NodeHandle n;

  std::string filename = "/home/rebecca/GITHUB/mast/ros_ws/mini_environment.txt";  
  environment_ = Tug::Environment(filename, 1.0, 0.01);
  ros::Publisher pub_goal = n.advertise<master::Waypoint>("goal_points", 1000);


  master::Waypoint pt1;
  pt1.ID = 1;
  pt1.x = 7;
  pt1.y = 10;
  pt1.v = 3;
  
  master::Waypoint pt2;
  pt2.ID = 2;
  pt2.x = 10;
  pt2.y = 0;
  pt2.v = 3;

  ros::Rate loop_rate(2);  
  while (ros::ok())
  {
  	pub_goal.publish(pt1);
    pub_goal.publish(pt2);
    loop_rate.sleep();
  }


  ros::spin();
  return 0;
}