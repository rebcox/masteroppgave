#include <ros/package.h>
#include <ros/ros.h>

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"
#include "tug_communicator.hpp"

#include "tugboat_control/Waypoint.h"
#include "tugboat_control/BoatPose.h"
#include "tug_constants.hpp"

/*namespace
{
  double ship_width_;
  double ship_length_;
  Tug::Environment environment_ship_;
  ros::Publisher ship_waypoint_pub;
  Tug::Boat ship_;
}*/

/*void callback_ship_pose(const tugboat_control::BoatPose::ConstPtr &msg)
{
  Tug::Point mid_pt(msg->x, msg->y, environment_ship_);

  bool waypoint_updated_flag;
  bool arrived_at_goal_flag;
  ship_.update_position(mid_pt, waypoint_updated_flag, arrived_at_goal_flag, ship_width_/2);
  Tug::Point pt_cur = ship_.get_current_waypoint();

  if(arrived_at_goal_flag)
  {
    tugboat_control::Waypoint waypoint;
    waypoint.x = pt_cur.x();
    waypoint.y = pt_cur.y();
    waypoint.v = 0;
    ship_waypoint_pub.publish(waypoint);
  }
  else //if (new_waypoint_set)
  {
    tugboat_control::Waypoint waypoint;
    waypoint.x = pt_cur.x();
    waypoint.y = pt_cur.y();
    waypoint.v = 5;
    ship_waypoint_pub.publish(waypoint);
  }
}*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "master_node");
  ros::NodeHandle node;
  
  if (argc < 2){ROS_ERROR("Need environment filename as argument"); return -1;};

  std::vector<std::string> all_args(argv, argv + argc);
  all_args.assign(argv + 1, argv + argc);
  std::string filename = all_args.at(0);

  try
  {
    Tug::Environment environment_tug = Tug::Environment(filename, 1, 0.01);
  }
  catch(...)
  {
    ROS_ERROR("File does not exist"); return -1;
  }

  Tug::Environment environment_tug = Tug::Environment(filename, 1, 0.01);

  //environment_tug.add_constant_safety_margin(0.2*SCALE);
  //environment_tug.mark_points_within_range(1.5);

  Tug::Communicator communicator(environment_tug, SCALE, 0.3); 

  ros::Subscriber sub_startup = node.subscribe("startup", 20, &Tug::Communicator::callback_new_tug, &communicator);
  ros::Subscriber sub_goal = node.subscribe("waypointRequest", 20, &Tug::Communicator::callback_waypoint, &communicator);
  ros::Subscriber sub_tug_locations = node.subscribe("pose", 20, &Tug::Communicator::callback_boat_pose, &communicator);
  ros::Subscriber sub_available_tugs = node.subscribe("waypTugs", 10, &Tug::Communicator::callback_available_tugs, &communicator);
  ros::Subscriber sub_clearWaypoint = node.subscribe("clearSingleWaypoint", 20, &Tug::Communicator::remove_end_point_from_planner, &communicator);
  ros::spin();

  return 0;
}