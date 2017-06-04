#include "tug_waypoint_publisher.hpp"
#include <stdexcept>
namespace Tug
{ 
  Waypoint_publisher::Waypoint_publisher()
  {
    wayp_pub = node_.advertise<master::Waypoint>("waypoint", 20);
    arrival_pub = node_.advertise<uInt8>("clearWaypoint", 20);
    client = node_.serviceClient<master::WaypointAvailable>("is_waypoint_available");
  }

  void Waypoint_publisher::update_position(const master::BoatPose::ConstPtr& msg)
  {
    newest_pose_ = *msg;

    master::Waypoint current_wp;
    if (current_waypoint_index_ >= path_.size())
    {
      return;
    }

    current_wp = path_[current_waypoint_index_];
    //Check if position is within radius of current waypoint
    if(sqrt(pow(newest_pose_.x - current_wp.x, 2) + pow(newest_pose_.y - current_wp.y, 2)) < acceptance_radius)
    {
      bool arrived_at_goal = go_to_next_waypoint();
    }
  }

  void Waypoint_publisher::set_path(const master::Path::ConstPtr &msg)
  {
    if (msg->tugID == id_)
    {
      path_.clear();
      path_ = msg.data;
      current_waypoint_index_= 0;
      msg->orderID = order_id_;
    }

  }

  void Waypoint_publisher::callback_update_goal(const master::Waypoint::ConstPtr &msg)
  {
    if (msg->ID == order_id_)
    {
      update_only_goal(msg);
    }
  }

  void Waypoint_publisher::update_only_goal(master::Waypoint goal)
  {
    path.pop_back();
    path.push_back(goal);

    if (no_waypoints_left == 1)
    {
      wayp_pub.publish(path_[current_waypoint_index_]); 
    }
  }

  bool Waypoint_publisher::is_waypoint_available(const master::Waypoint &pt)
  {
    master::WaypointAvailable srv;
    srv.request.waypoint = pt;

    if (client.call(srv))
    {
      bool is_available = srv.response.ans;
      return is_available;
    }
    ROS_WARN("Service WaypointAvailable failed to reply");
    return false;
  }

  bool Waypoint_publisher::go_to_next_waypoint()
  {
    ++current_waypoint_index_;
    if (current_waypoint_index_ >= path_.size())
    {
      --current_waypoint_index_;
      master::ClearWaypoint clear; clear.orderID = order_id_; clear.tugID = id_;
      //TODO: Sondre vil ha felles clear points
      arrival_pub.publish(id_);
      return true;
    }
    else
    {
      if (is_waypoint_available(path_[current_waypoint_index_]))
      {
        wayp_pub.publish(path_[current_waypoint_index_]);
      }
      else
      {
        --current_waypoint_index_;
      }
      return false;
    }
  }

  master::Waypoint Waypoint_publisher::get_current_waypoint()
  {
    if(current_waypoint_index_ < path_.size())
    {
      return path_[current_waypoint_index_];
    }
    else
    {
      master::Waypoint wp; wp.v = -1;
      return wp;
    }
  }

//TODO: ta hensyn til skip
/*
      if(tugs_.at(id).no_waypoints_left() == 2)
      {
        Tug::Polyline points_to_go_around_ship = route_around_ship_.best_route(pt_cur, tugs_.at(id).get_goal(), environment_tug_);
        
        if (points_to_go_around_ship.size() > 0)
        {
          points_to_go_around_ship.push_front(pt_cur);
          points_to_go_around_ship.push_back(tugs_.at(id).get_goal());
          tugs_.at(id).set_path(points_to_go_around_ship);
        }
      }


*/