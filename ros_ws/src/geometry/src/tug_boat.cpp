#include "tug_boat.hpp"
#include <stdexcept>

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

  void Boat::update_position(Point &position, bool &waypoint_updated_flag)
  {
    position_ = position;
    double radius = 1.0;
    waypoint_updated_flag = false;
    if(switching_point_decision_->accept(position, radius))
    {
      go_to_next_waypoint();
      waypoint_updated_flag = true;
    }
  }


  bool Boat::go_to_next_waypoint()
  {
    ++current_waypoint_index_;
    if (current_waypoint_index_ > path_.size())
    {
      return false;
    }
    else
    {
      set_switchingpoint_decision(path_[current_waypoint_index_-1], path_[current_waypoint_index_]);
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
  void Boat::set_switchingpoint_decision(Point &pt1, Point &pt2)
  {
    Switching_point_decision spd(pt1, pt2);
    switching_point_decision_ = &spd;
  }



}