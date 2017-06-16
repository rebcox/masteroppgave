#include "tug_waypoint_publisher.hpp"
#include <stdexcept>
namespace Tug
{ 

  Waypoint_publisher::Waypoint_publisher(int id, double scale)
  {
    id_ = id;
    scale_ = scale;
    acceptance_radius = 0.05;
    wayp_pub = node_.advertise<tugboat_control::Waypoint>("waypoint", 20);
    arrival_pub = node_.advertise<tugboat_control::ClearWaypoint>
                                ("clearSingleWaypoint", 20);
    client_is_avail = node_.serviceClient<tugboat_control::WaypointAvailable>
                                        ("isWaypointAvailable");
    client_avoid_ship = node_.serviceClient<tugboat_control::AvoidShipCollision>
                                          ("avoidShipCollision");
  }

  void Waypoint_publisher::update_position(const tugboat_control::BoatPose::ConstPtr& msg)
  {
    tugboat_control::BoatPose pose;
    pose.x = msg->x * scale_;
    pose.y = msg->y * scale_;
    pose.o = msg->o;
    newest_pose_ = pose;

    tugboat_control::Waypoint current_wp;
    if (current_waypoint_index_ >= path_.size())
    {
      return;
    }

    current_wp = path_[current_waypoint_index_];
    bool new_waypoint_set = false;
    //Check if position is within radius of current waypoint
    if(sqrt(pow(newest_pose_.x - current_wp.x, 2) + 
            pow(newest_pose_.y - current_wp.y, 2)) < acceptance_radius*scale_)
    {
      new_waypoint_set = go_to_next_waypoint();
    }
  }

  void Waypoint_publisher::set_path(const tugboat_control::Path::ConstPtr &msg)
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
      if (!SIMULATION)
      {
        call_path_around_ship_service(path_[0], path_[1], path_);
        current_waypoint_index_ = 0;
      }
    }

  }

  void Waypoint_publisher::call_path_around_ship_service(
                                           const tugboat_control::Waypoint &start,
                                           const tugboat_control::Waypoint &finish,
                                           std::vector<tugboat_control::Waypoint> &result)
  {
    tugboat_control::AvoidShipCollision srv; 
    srv.request.from = start;
    srv.request.to = finish;

    if(client_avoid_ship.call(srv))
    {
      result.clear();
      result = srv.response.path;
    }
    else
    {
      ROS_WARN("Service avoid_ship failed to reply");
    }
  }

  bool Waypoint_publisher::is_waypoint_available(const tugboat_control::Waypoint &pt)
  {
    //Waypoint available service not tested in real test setup
    if (!SIMULATION)
    {
      return true;
    }
    tugboat_control::WaypointAvailable srv;
    srv.request.waypoint = pt;
    srv.request.tugID = id_;

    if (client_is_avail.call(srv))
    {
      if (srv.response.ans.data == true)
      {
        return true;
      }
      else
      {
        return false;
      }  
    }
    ROS_WARN("Service WaypointAvailable failed to reply");
    return false;
  }

  void Waypoint_publisher::publish_current_waypoint()
  {
      tugboat_control::Waypoint wp;
      wp.x = path_[current_waypoint_index_].x/SCALE_OUT;
      wp.y = path_[current_waypoint_index_].y/SCALE_OUT;
      wp.ID = id_;
      wp.v = TUG_SPEED;
      wayp_pub.publish(wp);
  }

  bool Waypoint_publisher::go_to_next_waypoint()
  {
    ++current_waypoint_index_;

    if (current_waypoint_index_ >= path_.size())
    {
      ROS_WARN("Tug %d has arrived", id_);
      --current_waypoint_index_;
      tugboat_control::ClearWaypoint clear; clear.orderID = order_id_; clear.tugID = id_;
      arrival_pub.publish(clear);
      is_already_on_hold = false;
      return false;
    }
    //heading towards goal
    else if (!SIMULATION && current_waypoint_index_ == path_.size() - 1) 
    {
      ROS_WARN("Trying to find route around ship if it is on the wrong side");

      call_path_around_ship_service(path_[current_waypoint_index_-1],
                                    path_[path_.size() - 1], path_);
      current_waypoint_index_ = 1;
      publish_current_waypoint();
      return true;
    }
    else
    {
      ROS_WARN("Tug %d arrived at a waypoint", id_);

      if (is_waypoint_available(path_[current_waypoint_index_]))
      {
        publish_current_waypoint();
        is_already_on_hold = false;
        return true;
      }
      else
      {
        --current_waypoint_index_;
        if (!is_already_on_hold)
        {
          ROS_WARN("Tug %d on hold", id_);
          is_already_on_hold = true;
        }
        return false;
      } 
    }
  }
}