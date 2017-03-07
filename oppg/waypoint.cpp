
#include "include/waypoint.h"

namespace Tug
{ 
  Waypoint::Waypoint(const VisiLibity::Point &point)
  {
    point_ = point;
  }

  bool Waypoint::is_within_safety_region()
  {
    return true;
  }

  float Waypoint::get_radius()
  {
    return radius_;
  }
  float Waypoint::get_speed()
  {
    return speed_;
  }
  
  void Waypoint::possible_angle_range(VisiLibity::Angle &from_angle, VisiLibity::Angle &to_angle)
  {

  }


}
