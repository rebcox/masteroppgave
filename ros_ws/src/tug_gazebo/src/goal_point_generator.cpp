#include "ros/package.h"
#include "ros/ros.h"

#include "geometry/tug_environment.hpp"
#include <tugboat_control/Waypoint.h>
#include <tugboat_control/ClearWaypoint.h>
#include <string>

namespace
{
  std::map<int, tugboat_control::Waypoint> waypoints;
}

void callback_clear_waypoint(const tugboat_control::ClearWaypoint::ConstPtr& msg)
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

  ros::Publisher pub_goal = n.advertise<tugboat_control::Waypoint>("waypointRequest", 20);
  ros::Subscriber sub_arrived = n.subscribe("clearWaypoint", 20, callback_clear_waypoint);

  tugboat_control::Waypoint pt1;
  pt1.ID = 11;
  pt1.x = 2;
  pt1.y = 1.625;

  tugboat_control::Waypoint pt2;
  pt2.ID = 22;
  pt2.x = 2.5;
  pt2.y = 1.75;

  waypoints.insert(std::pair<int, tugboat_control::Waypoint>(pt1.ID, pt1));
  waypoints.insert(std::pair<int, tugboat_control::Waypoint>(pt2.ID, pt2));



  ros::Rate loop_rate(2);  
  while (ros::ok())
  {
    for (std::map<int, tugboat_control::Waypoint>::iterator pt = waypoints.begin(); pt != waypoints.end(); ++pt)
    {
      pub_goal.publish(pt->second);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }


  ros::spin();
  return 0;
}