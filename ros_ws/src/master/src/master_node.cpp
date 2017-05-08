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
  std::map<int, Tug::Point> current_waypoints_; //int is for tug_id
  std::map<int, std::vector<int>> tugs_holding_other_tugs_;
  //std::vector<master::BoatPose> position_msgs_(11);
}


float eucledian_distance(const Tug::Point &point1, const Tug::Point &point2)
{
  return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
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

void remove_end_point_from_planner(Tug::Point &pt)
{
  for (std::vector<Tug::Point>::iterator i = end_points_.begin(); i != end_points_.end(); ++i)    
  {
    if (*i == pt)
    {
      end_points_.erase(i);
    }
  }
  Tug::Assign_paths assigner;
  assigner.assign_on_combined_shortest_path(tugs_, end_points_, environment_); 
}

void publish_new_waypoint(const Tug::Point *pt_cur, int tug_id)
{
  master::Waypoint waypoint;
  waypoint.ID = tug_id;
  waypoint.x = pt_cur->x();
  waypoint.y = pt_cur->y();
  waypoint.v = 1;

  bool tug_in_the_way = false;

  //Check for tugs in the way of point
  for (int i = 0; i < current_waypoints_.size(); ++i)
  {
    if (tug_id == i)
    {
      continue;
    }
    if (current_waypoints_[i] == *pt_cur || eucledian_distance(current_waypoints_[i],*pt_cur) < 10) //TODO: hardkoda range
    {
      waypoint.v = 0;
      tugs_holding_other_tugs_.at(i).push_back(tug_id);
      tug_in_the_way = true;
    }  
  }

  pub.publish(waypoint);
  
  //if new point with speed > 0 will be published
  //check if the tug had other tug dependent
  if (!tug_in_the_way)
  {
    current_waypoints_.at(tug_id) = *pt_cur;
   // for (int i = 0; i < tugs_holding_other_tugs_.at(tug_id).size(); ++i)
    for (std::vector<int>::iterator i = tugs_holding_other_tugs_.at(tug_id).begin(); i != tugs_holding_other_tugs_.at(tug_id).end(); ++i)
    {
      //int tug = tugs_holding_other_tugs_.at(tug_id).at(i);
      int tug = *i;
      tugs_holding_other_tugs_.at(tug_id).erase(i);
      publish_new_waypoint(tugs_.at(tug).get_current_waypoint(), tug);
    }
    //tugs_holding_other_tugs_.at(tug_id).clear();
  }
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
    Tug::Point* pt_cur = tugs_.at(id).get_current_waypoint();
    publish_new_waypoint(pt_cur, id);
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
  environment_.mark_points_within_range(10.0);

  //TODO: Add tugs to tugs_


  ros::Subscriber sub_goal = n.subscribe("sondres_goal_points", 1000, callback_waypoint);
  ros::Subscriber sub_tug_locations = n.subscribe("sondres_tug_locations", 1000, callback_boat_pose);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}