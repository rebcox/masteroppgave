#include "include/tug_boat.hpp"
#include <stdexcept>

namespace Tug
{
  Boat::Boat(double length, double width)
  {
    if (length >= 0 && width >= 0)
    {
      length_ = length;
      width_ = width;
    }
    else
    {
      throw std::invalid_argument( "received negative value of length or width" );
    }
  }
}