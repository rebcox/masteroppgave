#include "ros/package.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry/tug_environment.hpp"
#include "geometry/tug_boat.hpp"
#include "search/tug_shortest_path.hpp"
#include "coordination/tug_assign_paths.hpp"
#include <master/Waypoint.h>
#include <master/BoatPose.h>
#include <sstream>
#include <string>

namespace
{
  std::map<int, Tug::Boat> tugs_;
  Tug::Environment environment_;
  std::vector<Tug::Point> end_points_;
  ros::Publisher pub;
  //std::vector<master::BoatPose> position_msgs_(11);
}

void print_path(Tug::Polyline &path)
{
  ROS_INFO("length of path: %f", path.length());
  std::stringstream path_string;
  for (int i = 0; i < path.size(); ++i)
  {
    path_string << path[i];
    if (i < path.size()-1)
    {
      path_string << " - ";
    }
  }
  ROS_INFO("Shortest path: %s", path_string.str().c_str());
}

/*void update_tug_positions()
{
  for (std::map<int, Tug::Boat>::iterator i = tugs_.begin(); i != tugs_.end(); ++i)
  {
    master::BoatPose msg = position_msgs_.at(i->first); //i->first = id
    Tug::Point pt(msg.x, msg.y, environment_);
    i->second.set_position(pt);
  }
}*/

void add_end_point_to_planner(Tug::Point &pt)
{
  end_points_.push_back(pt);
  Tug::Assign_paths assigner;
  //update_tug_positions();
  assigner.assign_on_combined_shortest_path(tugs_, end_points_, environment_); 
}

void publish_new_waypoint(const Tug::Point *pt, int tug_id)
{
  master::Waypoint waypoint;
  waypoint.ID = tug_id;
  waypoint.x = pt->x();
  waypoint.y = pt->y();

  pub.publish(waypoint);
}

void callback_waypoint(const master::Waypoint::ConstPtr& msg)
{
  //Add waypoint
  Tug::Point pt(msg->x, msg->y, environment_);
  add_end_point_to_planner(pt);
}

void callback_boat_pose(const master::BoatPose::ConstPtr& msg)
{
  int id = msg->ID;
  //position_msgs_.at(id) = *msg;
  Tug::Point pt(msg->x, msg->y, environment_);
  bool new_waypoint_set;
  tugs_.at(id).update_position(pt, new_waypoint_set);

  if (new_waypoint_set)
  {
    Tug::Point* pt = tugs_.at(id).get_current_waypoint();
    publish_new_waypoint(pt, id);
  }
}


/*subscribe på Sondres "få tug hit"
hver gang det dukker opp en ny
  for alle båter som ikke har nådd sin destinasjon
    - rekalkuler munkres og scheduler*/


int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;
  pub = n.advertise<master::Waypoint>("goTo", 1000);

  std::string filename = "/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt";
  environment_ = Tug::Environment(filename, 1.0, 0.01);
  //TODO: Add tugs

  ros::Subscriber sub_goal = n.subscribe("sondres_goal_points", 1000, callback_waypoint);
  ros::Subscriber sub_tug_locations = n.subscribe("sondres_tug_locations", 1000, callback_boat_pose);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}