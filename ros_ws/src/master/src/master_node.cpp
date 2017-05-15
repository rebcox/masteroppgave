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

void publish_new_waypoint(const Tug::Point *pt_cur, int tug_id);

namespace
{
  std::map<int, Tug::Boat> tugs_;
  Tug::Environment environment_;
  std::vector<Tug::Point> end_points_;
  ros::Publisher pub;
  std::map<int, Tug::Point> current_waypoints_; //int is for tug_id
  std::map<int, std::vector<int>> tugs_holding_other_tugs_;
}

float eucledian_distance(const Tug::Point &point1, const Tug::Point &point2)
{
  return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
}

void print_path(Tug::Polyline path)
{
  //ROS_INFO("length of path: %f", path.length());
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

void add_end_point_to_planner(const Tug::Point &pt)
{
  ROS_INFO("end point set to (%f, %f)", pt.x(), pt.y());
  end_points_.push_back(pt);
  Tug::Assign_paths assigner;
  //update_tug_positions();

  assigner.assign_on_combined_shortest_path(tugs_, end_points_, environment_); 
  for (std::map<int, Tug::Boat>::iterator tug = tugs_.begin(); tug != tugs_.end(); ++tug)
  {
    print_path(tug->second.get_path());
  }
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
  ROS_ERROR("YO");

  master::Waypoint waypoint;
  waypoint.ID = tug_id;
  waypoint.x = pt_cur->x();
  waypoint.y = pt_cur->y();
  waypoint.v = 3;

  bool another_tug_in_the_way = false;

  //Check for tugs in the way of point
  for (std::map<int, Tug::Point>::iterator pt = current_waypoints_.begin(); pt != current_waypoints_.end(); ++pt)
  {
    if (pt->first == tug_id)
     {
       continue;
     } 
     //if a current waypoint is within a certain range of the point to be published, publish it with v=0
     if (pt->second == *pt_cur ||  eucledian_distance(pt->second, *pt_cur) < 0.1) //TODO: hardkoda range
     {
        waypoint.v = 0;
        ROS_INFO("tug %d is held by tug %d", tug_id, pt->first);
        tugs_holding_other_tugs_.at(pt->first).push_back(tug_id);
        another_tug_in_the_way = true;
     }
  }

  ROS_INFO("Published new point (%f, %f)", pt_cur->x(), pt_cur->y());

  pub.publish(waypoint);
  
  //if new point with speed > 0 will be published,
  //check if the tug had other tugs dependent on it
  if (!another_tug_in_the_way)
  {
    //set current_waypoints_
    current_waypoints_.at(tug_id) = *pt_cur;
    //Looping through all tugs the new published point were holding
    for (std::vector<int>::iterator i = tugs_holding_other_tugs_.at(tug_id).begin();
           i != tugs_holding_other_tugs_.at(tug_id).end(); 
              ++i)
    {
      //int tug = tugs_holding_other_tugs_.at(tug_id).at(i);
      tugs_holding_other_tugs_.at(tug_id).erase(i);
      publish_new_waypoint(tugs_.at(*i).get_current_waypoint(), *i);
    }
    //tugs_holding_other_tugs_.at(tug_id).clear();
  }
}

bool is_new(const Tug::Point &end_point)
{
  for (std::vector<Tug::Point>::iterator pt = end_points_.begin(); pt != end_points_.end(); ++pt)
  {
    if (*pt == end_point)
    {
      return false;
    }
  }
  return true;
}

void callback_waypoint(const master::Waypoint::ConstPtr& msg)
{  
  Tug::Point pt(msg->x, msg->y, environment_);

  if (is_new(pt))
  {
    ROS_INFO("Call to add_end_point_to_planner");
    add_end_point_to_planner(pt);
  } 
}

void callback_boat_pose(const master::BoatPose::ConstPtr& msg)
{
  ROS_ERROR("callback_boat_pose kalt");
  if (end_points_.size() == 0)
  {
    return;
  }

  int id = msg->ID;
  Tug::Point pt(msg->x, msg->y, environment_);
  bool new_waypoint_set;

  tugs_.at(id).update_position(pt, new_waypoint_set, 0.5);

  Tug::Point* pt_cur = tugs_.at(id).get_current_waypoint();
  if (!pt_cur)
  {
    return;
  }
  //ROS_INFO("current wp (%f, %f)", pt_cur->x(), pt_cur->y());

  if (new_waypoint_set)
  {
    publish_new_waypoint(pt_cur, id);
  }
  else
  {

    master::Waypoint waypoint;
    waypoint.ID = id;
    waypoint.x = pt_cur->x();
    waypoint.y = pt_cur->y();
    waypoint.v = 3;
    //ROS_INFO("Published point (%f, %f)", pt_cur->x(), pt_cur->y());

    pub.publish(waypoint);
  }

}

void add_new_tug(Tug::Boat &tug, int id)
{
  tug.set_id(id);
  tugs_.insert(std::pair<int, Tug::Boat>(id, tug));
  current_waypoints_.insert(std::pair<int, Tug::Point>(id, tug.get_position()));
  tugs_holding_other_tugs_.insert(std::pair<int, std::vector<int>>(id, std::vector<int>()));
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "master_node");
  ros::NodeHandle node;
  pub = node.advertise<master::Waypoint>("goTo", 1000);

  //std::string filename = "/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt";
  std::string filename = "/home/rebecca/GITHUB/mast/ros_ws/mini_environment.txt";

  environment_ = Tug::Environment(filename, 1.0, 0.01);
  environment_.mark_points_within_range(1.5);

  //TODO: Add tugs to tugs_
  Tug::Point pt1(0.0,0.0, environment_);
  Tug::Boat tug1(1, pt1 , &environment_);
  add_new_tug(tug1, 1);

  Tug::Point pt2(5.0, 0.0, environment_);
  Tug::Boat tug2(1, pt2 , &environment_);
  add_new_tug(tug2, 2);


  ros::Subscriber sub_goal = node.subscribe("goal_points", 1000, callback_waypoint);
  ros::Subscriber sub_tug_locations = node.subscribe("tug_locations", 1000, callback_boat_pose);

  ros::spin();

  return 0;
}