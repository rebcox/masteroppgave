#include "tug_waypoint_publisher.hpp"
#include <stdexcept>
namespace Tug
{ 

  Waypoint_publisher::Waypoint_publisher(int id, double scale)
  {
    id_ = id;
    scale_ = scale;
    //acceptance_radius = 0.05*scale;
    wayp_pub = node_.advertise<master::Waypoint>("waypoint", 20);
    arrival_pub = node_.advertise<master::ClearWaypoint>("clearSingleWaypoint", 20);
    client_is_avail = node_.serviceClient<master::WaypointAvailable>("is_waypoint_available");
    client_avoid_ship = node_.serviceClient<master::AvoidShipCollision>("avoid_ship_collision");
  }

  void Waypoint_publisher::update_position(const master::BoatPose::ConstPtr& msg)
  {
    master::BoatPose pose;
    pose.x = msg->x * scale_;
    pose.y = msg->y * scale_;
    pose.o = msg->o;
    newest_pose_ = pose;

    master::Waypoint current_wp;
    if (current_waypoint_index_ >= path_.size())
    {
      return;
    }

    current_wp = path_[current_waypoint_index_];
    bool arrived_at_goal;
    //Check if position is within radius of current waypoint
    if(sqrt(pow(newest_pose_.x - current_wp.x, 2) + pow(newest_pose_.y - current_wp.y, 2)) < acceptance_radius*scale_)
    {
      //ROS_WARN("arrived at wp");
      arrived_at_goal = go_to_next_waypoint();
      //ROS_WARN("goint towards next wp");
    }
    if (!arrived_at_goal)
    {
      wayp_pub.publish(path_[current_waypoint_index_]);
    }
  }

  void Waypoint_publisher::set_path(const master::Path::ConstPtr &msg)
  {
    if (msg->tugID == id_)
    {
      path_.clear();
      path_ = msg->data;
      current_waypoint_index_= 0;
      order_id_ = msg->orderID;
    }
    if (path_.size() == 2)
    {
      master::AvoidShipCollision srv;
      srv.request.from = path_[0];
      srv.request.to = path_[1];

      if(client_avoid_ship.call(srv))
      {
        path_.clear();
        path_ = srv.response.path;
      }
      else
      {
        ROS_WARN("Service avoid_ship failed to reply");
      }
    }

  }

  void Waypoint_publisher::callback_update_goal(const master::Waypoint::ConstPtr &msg)
  {
    if (msg->ID == order_id_)
    {
      update_only_goal(*msg);
    }
  }

  void Waypoint_publisher::update_only_goal(master::Waypoint goal)
  {
    if (path_.size() > 0)
    {
      path_.pop_back();
      path_.push_back(goal);

      if (no_waypoints_left() == 1)
      {
        wayp_pub.publish(path_[current_waypoint_index_]); 
      }
    }
  }

  bool Waypoint_publisher::is_waypoint_available(const master::Waypoint &pt)
  {
    master::WaypointAvailable srv;
    srv.request.waypoint = pt;

    if (client_is_avail.call(srv))
    {
      if (srv.response.ans.data == true)
      {
        ROS_INFO("service reply true");
        return true;
      }
      else
      {
        ROS_INFO("service reply false");
        return false;
      }  
    }
    ROS_WARN("Service WaypointAvailable failed to reply");
    return false;
  }

  void Waypoint_publisher::wait_at_current_point()
  {
    master::Waypoint pt;
    pt.x = path_[current_waypoint_index_].x;
    pt.y = path_[current_waypoint_index_].y;
    pt.v = 0;

    wayp_pub.publish(path_[current_waypoint_index_]);
  }

  bool Waypoint_publisher::go_to_next_waypoint()
  {
    ++current_waypoint_index_;
    if (current_waypoint_index_ >= path_.size())
    {
      --current_waypoint_index_;
      master::ClearWaypoint clear; clear.orderID = order_id_; clear.tugID = id_;
     // ROS_INFO("Tug %d arrived at goal", id_);
      wait_at_current_point();
      arrival_pub.publish(clear);
      return true;
    }
    else if (current_waypoint_index_ == path_.size() - 2)
    {
      master::AvoidShipCollision srv;
      srv.request.from = path_[path_.size() - 2];
      srv.request.to = path_[path_.size() - 1];

      if(client_avoid_ship.call(srv))
      {
        path_.clear();
        path_ = srv.response.path;
        current_waypoint_index_ = 0;
        wayp_pub.publish(path_[current_waypoint_index_]);
      }
      else
      {
        ROS_WARN("Service avoid_ship failed to reply");
      }
    }
    else
    {
      if (is_waypoint_available(path_[current_waypoint_index_]))
      {
        ROS_INFO("Published waypoint (%f, %f) for tug %d",path_[current_waypoint_index_].x, path_[current_waypoint_index_].y, id_);

        wayp_pub.publish(path_[current_waypoint_index_]);
      }
      else
      {
        --current_waypoint_index_;
        wait_at_current_point();
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
}