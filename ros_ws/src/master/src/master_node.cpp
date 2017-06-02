#include <ros/package.h>
#include <ros/ros.h>

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"
#include "tug_communicator.hpp"

#include "master/Waypoint.h"
#include "master/BoatPose.h"

namespace
{
  double ship_width_;
  double ship_length_;
  Tug::Environment environment_ship_;
  ros::Publisher ship_waypoint_pub;
  Tug::Boat ship_;
}

void callback_ship_pose(const master::BoatPose::ConstPtr &msg)
{
  Tug::Point mid_pt(msg->x, msg->y, environment_ship_);

  bool waypoint_updated_flag;
  bool arrived_at_goal_flag;
  ship_.update_position(mid_pt, waypoint_updated_flag, arrived_at_goal_flag, ship_width_/2);
  Tug::Point pt_cur = ship_.get_current_waypoint();

  if(arrived_at_goal_flag)
  {
    master::Waypoint waypoint;
    waypoint.x = pt_cur.x();
    waypoint.y = pt_cur.y();
    waypoint.v = 0;
    ship_waypoint_pub.publish(waypoint);
  }
  else //if (new_waypoint_set)
  {
    master::Waypoint waypoint;
    waypoint.x = pt_cur.x();
    waypoint.y = pt_cur.y();
    waypoint.v = 5;
    ship_waypoint_pub.publish(waypoint);
  }
}

double meter_to_pixel(double meter, double scale)
{
  return meter*scale;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "master_node");
  ros::NodeHandle node;
  ship_waypoint_pub = node.advertise<master::Waypoint>("shipWaypoint", 10);

  double scale = 40.0;

  std::string filename = "/home/rebecca/GITHUB/mast/ros_ws/big_test.txt";

  //double px = 1.0/200.0;
  Tug::Environment environment_tug = Tug::Environment(filename, 1, 0.01);

  //environment_tug_.add_constant_safety_margin(1*scale);
  environment_tug.mark_points_within_range(1.5);

  Tug::Point tug1_location(0.0*scale,0.0*scale, environment_tug);
  Tug::Boat tug1(1, tug1_location , &environment_tug);

  Tug::Point tug2_location(2.0*scale, 0.0*scale, environment_tug);
  Tug::Boat tug2(1, tug2_location , &environment_tug);

  std::map<int,Tug::Boat> tugs;
  tugs.insert(std::pair<int,Tug::Boat>(1, tug1));
  tugs.insert(std::pair<int,Tug::Boat>(2, tug2));

  Tug::Communicator communicator(environment_tug, scale, 0.3); 

  environment_ship_ = Tug::Environment(filename, 1.0, 0.01);
  Tug::Point ship_goal(2, 2, environment_ship_);
  ship_width_ = 2;
  ship_length_ = 4;

  Tug::Point start_location_ship(0, 0, environment_ship_);
  ship_ = Tug::Boat(1, start_location_ship, &environment_ship_);
  Tug::Polyline shortest_path_ship;

  Tug::Shortest_path sp(environment_ship_, start_location_ship, ship_goal, shortest_path_ship);
  ship_.set_path(shortest_path_ship);

  ros::Subscriber sub_ship_loc_for_ship_planning = node.subscribe("shipPose", 1, callback_ship_pose);

  ros::Subscriber sub_startup = node.subscribe("startup", 20, &Tug::Communicator::callback_new_tug, &communicator);
  ros::Subscriber sub_goal = node.subscribe("waypointRequest", 20, &Tug::Communicator::callback_waypoint, &communicator);
  ros::Subscriber sub_tug_locations = node.subscribe("pose", 20, &Tug::Communicator::callback_boat_pose, &communicator);
  ros::Subscriber sub_ship_locations = node.subscribe("shipPose", 1, &Tug::Communicator::callback_ship_pose, &communicator);
  ros::Subscriber sub_available_tugs = node.subscribe("waypTugs", 1, &Tug::Communicator::callback_available_tugs, &communicator);

  ros::spin();

  return 0;
}