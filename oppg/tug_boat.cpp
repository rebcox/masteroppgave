#include "include/tug_boat.hpp"
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
  void Boat::set_path(const Point &finish)
  {
    path_ = environment_->shortest_path(position_, finish);
  }
}