#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"
#include "tug_communicator.hpp"
#include "tug_constants.hpp"
#include "tugboat_control/BoatPose.h"
#include "tugboat_control/Waypoint.h"

#include <ros/package.h>
#include <ros/ros.h>

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

  Tug::Communicator communicator(environment_tug, SCALE, 0.3); 

  ros::Subscriber sub_startup = node.subscribe("startup", 
                                               20, 
                                               &Tug::Communicator::callback_new_tug, 
                                               &communicator);
  ros::Subscriber sub_goal = node.subscribe("waypointRequest",
                                            20, 
                                            &Tug::Communicator::callback_waypoint, 
                                            &communicator);

  ros::Subscriber sub_tug_locations = node.subscribe("pose", 
                                                     20, 
                                                     &Tug::Communicator::callback_boat_pose, 
                                                     &communicator);
  ros::Subscriber sub_available_tugs = 
                        node.subscribe("waypTugs", 
                        10, 
                        &Tug::Communicator::callback_available_tugs, 
                        &communicator);
  ros::Subscriber sub_clearWaypoint = 
                         node.subscribe("clearSingleWaypoint", 
                         20, 
                         &Tug::Communicator::remove_end_point_from_planner, &communicator);
  ros::spin();

  return 0;
}