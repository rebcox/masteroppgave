#include "tug_boat.hpp"
#include <stdexcept>
#include "ros/ros.h"
namespace Tug
{
  Boat::Boat(double radius, const Point &position, Environment *environment)
  {
    if (radius > 0)
    {
      radius_ = radius;
      position_ = position;
      environment_ = environment;
    }
    else
    {
      throw std::invalid_argument( "received negative value of length or width" );
    }
  }
  
  Point Boat::get_position()
  {
    return position_;
  }

  void Boat::update_position(Point &position, bool &waypoint_updated_flag, double radius)
  {
    position_ = position;

    waypoint_updated_flag = false;
    //if(switching_point_decision_->accept(position, radius))
    try
    {
      Point *finish;
      if (current_waypoint_index_ >= path_.size())
      {
        return;
      }
      finish = &path_[current_waypoint_index_];

      if(sqrt(pow(position.x() - finish->x(), 2) + pow(position.y() - finish->y(), 2)) < radius)
      {
        waypoint_updated_flag = go_to_next_waypoint();
      }
    }
    catch(...)
    {
      std::cerr << "Error in Boat::update_position" << std::endl;
      return;
    }
    

  }

  void Boat::set_path(const Polyline &path)
  {
    path_ = path;
    current_waypoint_index_= 0;
    go_to_next_waypoint();    
  }

  bool Boat::go_to_next_waypoint()
  {
    ++current_waypoint_index_;
    if (current_waypoint_index_ >= path_.size())
    {
      --current_waypoint_index_;
      return false;
    }
    else
    {
      //set_switchingpoint_decision(path_[current_waypoint_index_-1], path_[current_waypoint_index_]);
      return true;
    }
  }

  Point *Boat::get_current_waypoint()
  {
    try
    {
      return &path_[current_waypoint_index_];
    }
    catch(...)
    {
      return nullptr;
    }
  }

  Point *Boat::get_previous_waypoint()
  {
    try
    {
      return &path_[current_waypoint_index_-1];
    }
    catch(...)
    {
      return nullptr;
    }
  }

  /*void Boat::set_switchingpoint_decision(Point &pt1, Point &pt2)
  {
    ROS_ERROR("Switching_point_decision satt: (%f, %f) og (%f, %f)", pt1.x(), pt1.y(), pt2.x(), pt2.y());
    Switching_point_decision spd(pt1, pt2);
    switching_point_decision_ = &spd;
  }*/



}