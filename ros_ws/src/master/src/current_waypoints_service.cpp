#include "ros/ros.h"
#include "master/WaypointAvailable.h"
#include "std_msgs/Bool.h"
#include "master/Waypoint.h"


namespace
{
  std::map<int, master::Waypoint> waypoints_;
}

void set_waypoint(const master::Waypoint::ConstPtr &msg)
{
  int id = msg->ID;
  try
  {
    waypoints_.at(id) = *msg;
  }
  catch(const std::out_of_range &oor)
  {
    waypoints_.insert(std::pair<int,master::Waypoint>(id, *msg));
  }
}

bool check(master::WaypointAvailable::Request &req,
           master::WaypointAvailable::Response &res)
{
  for (std::map<int, master::Waypoint>::iterator i = waypoints_.begin(); i != waypoints_.end(); ++i)
  {
    if (req.waypoint.x == i->second.x && req.waypoint.y == i->second.y)
    {
      res.ans.data = false;
      return true;
    }
  }
  res.ans.data = true;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "current_waypoint_service");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("waypoint", 20, set_waypoint);
  ros::ServiceServer service = node.advertiseService("is_waypoint_available", check);

  ros::spin();
  return 0;
}