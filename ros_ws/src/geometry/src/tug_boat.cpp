#include "tug_boat.hpp"

#include <ros/ros.h>
#include <stdexcept>

namespace Tug
{ 
  Point Boat::get_position()
  {
    return position_;
  }

  void Boat::update_position(Point &position)
  {
    position_ = position;
  }

  void Boat::set_path(const Polyline &path)
  {
    path_.clear();
    path_ = path;   
  }
}