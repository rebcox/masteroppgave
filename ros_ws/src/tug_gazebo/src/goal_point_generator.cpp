#include "ros/package.h"
#include "ros/ros.h"

#include "geometry/tug_environment.hpp"
#include <master/Waypoint.h>
#include <master/ClearWaypoint.h>
#include <string>

Tug::Environment environment_;

namespace
{
  std::map<int, master::Waypoint> waypoints;
}

void callback_clear_waypoint(const master::ClearWaypoint::ConstPtr& msg)
{
  try
  {
    waypoints.erase(msg->orderID);
  }
  catch(const std::out_of_range &oor){}
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "goal_points_generator");
  ros::NodeHandle n;

  std::string filename = "/home/rebecca/GITHUB/mast/ros_ws/mini_environment.txt";  
  environment_ = Tug::Environment(filename, 1.0, 0.01);
  ros::Publisher pub_goal = n.advertise<master::Waypoint>("waypointRequest", 20);

  ros::Subscriber sub_arrived = n.subscribe("clearWaypoint", 20, callback_clear_waypoint);

 /* master::Waypoint pt1;
  pt1.ID = 11;
  pt1.x = 7;
  pt1.y = 10;
  pt1.v = 3;

  master::Waypoint pt2;
  pt2.ID = 22;
  pt2.x = 8.5;
  pt2.y = 10;
  pt2.v = 3;*/

  master::Waypoint pt1;
  pt1.ID = 11;
  pt1.x = 2;
  pt1.y = 1.625;

  master::Waypoint pt2;
  pt2.ID = 22;
  pt2.x = 2.5;
  pt2.y = 1.75;

  waypoints.insert(std::pair<int, master::Waypoint>(pt1.ID, pt1));
  waypoints.insert(std::pair<int, master::Waypoint>(pt2.ID, pt2));



  ros::Rate loop_rate(2);  
  while (ros::ok())
  {
    for (std::map<int, master::Waypoint>::iterator pt = waypoints.begin(); pt != waypoints.end(); ++pt)
    {
      pub_goal.publish(pt->second);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }


  ros::spin();
  return 0;
}