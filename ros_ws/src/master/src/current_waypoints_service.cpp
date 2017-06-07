#include "ros/ros.h"
#include "tugboat_control/WaypointAvailable.h"
#include "std_msgs/Bool.h"
#include "tugboat_control/Waypoint.h"

#define SCALE 220
#define CLOSE_POINTS_RADIUS 0.1

namespace
{
  std::map<int, tugboat_control::Waypoint> waypoints_;
}

void set_waypoint(const tugboat_control::Waypoint::ConstPtr &msg)
{
  int id = msg->ID;
  try
  {
    tugboat_control::Waypoint scaledpt;
    scaledpt.x = msg->x*SCALE;
    scaledpt.y = msg->y*SCALE;
    waypoints_.at(id) = scaledpt;
  }
  catch(const std::out_of_range &oor)
  {
    tugboat_control::Waypoint scaledpt;
    scaledpt.x = msg->x*SCALE;
    scaledpt.y = msg->y*SCALE;
    waypoints_.insert(std::pair<int,tugboat_control::Waypoint>(id, scaledpt));
  }
}

bool check(tugboat_control::WaypointAvailable::Request &req,
           tugboat_control::WaypointAvailable::Response &res)
{
  for (int i = 0; i < waypoints_.size(); ++i)
  {
    ROS_INFO("wp %d: (%f, %f)", i, waypoints_[i].x, waypoints_[i].y);
  }

  for (std::map<int, tugboat_control::Waypoint>::iterator i = waypoints_.begin(); i != waypoints_.end(); ++i)
  {
    ROS_INFO("dist between points: %f", sqrt(pow(req.waypoint.x - i->second.x, 2) + pow(req.waypoint.y - i->second.y,2)));
    if (sqrt(pow(req.waypoint.x - i->second.x, 2) + pow(req.waypoint.y - i->second.y,2)) < CLOSE_POINTS_RADIUS*SCALE)
    {
      ROS_WARN("Tug %d is tring to approach a waypoint too close to antoher tug ", req.waypoint.ID);
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