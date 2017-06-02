#include <ros/package.h>
#include <ros/ros.h>

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"
#include "tug_communicator.hpp"

#include "master/Waypoint.h"
#include "master/BoatPose.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_tester");
  ros::NodeHandle node;

  double scale = 24; //480/2;

  std::string filename = "/home/rebecca/GITHUB/mast/ros_ws/big_test.txt";
  Tug::Environment environment_tug = Tug::Environment(filename, 1, 0.01);

  environment_tug.mark_points_within_range(1.5);

  Tug::Communicator communicator(environment_tug, scale);

  ros::Subscriber sub_startup = node.subscribe("startup", 20, &Tug::Communicator::callback_new_tug, &communicator);
  ros::Subscriber sub_goal = node.subscribe("waypointRequest", 20, &Tug::Communicator::callback_waypoint, &communicator);
  ros::Subscriber sub_tug_locations = node.subscribe("pose", 20, &Tug::Communicator::callback_boat_pose, &communicator);
  ros::Subscriber sub_ship_locations = node.subscribe("shipPose", 1, &Tug::Communicator::callback_ship_pose, &communicator);
  ros::Subscriber sub_available_tugs = node.subscribe("waypTugs", 1, &Tug::Communicator::callback_available_tugs, &communicator);

  ros::spin();

  return 0;
}