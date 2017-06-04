#include "ros.h"
#include "master/WaypointAvailable.h"
#include "std_msgs/Bool.h"


namespace
{
  std::map<int, master::Waypoint> waypoints_;
  //ros::Publisher waypoints_pub = node_.advertise<master::Waypoints>("current_waypoints", 20);

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
/*
  master::Waypoints waypoints_msg;
  for (std::map<int, master::Waypoints>::iterator i = waypoints_.begin(); i != waypoints_.end(); ++i)
  {
    waypoints_msg.data.push_back(i->second);
  }
  waypoints_.publish(waypoints_msg);*/
}

bool check(master::WaypointAvailable::Request &req,
           WaypointAvailable::Response &res)
{
  for (std::map<int, master::waypoint>::iterator i = waypoints_.begin(); i != waypoints_.end(); ++i)
  {
    if (req->x == i->second.x && req->y == i->second.y)
    {
      res->ans = false;
      return true;
    }
  }
  res->ans = true;
  return true;
}


int main(int argc, char const *argv[])
{
  ros::init(argc, argv, "current_waypoint_service");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("waypoint", 20, set_waypoint);
  ros::ServiceServer service = node.advertiseService("is_waypoint_available", check);

  return 0;
}