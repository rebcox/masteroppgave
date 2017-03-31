#include "include/tug_boat.hpp"
#include <stdexcept>

namespace Tug
{
  Boat::Boat(double radius, const Point &position)
  {
    if (radius > 0)
    {
      radius_ = radius;
      position_ = position;
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
}