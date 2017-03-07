#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "visilibity.hpp"

namespace Tug
{
  class Waypoint
  {
  public:
    Waypoint(const VisiLibity::Point &point);
    bool is_within_safety_region();
    float get_radius();
    float get_speed();
    void possible_angle_range(VisiLibity::Angle &from_angle, VisiLibity::Angle &to_angle);
  private:
    VisiLibity::Point point_;
    float radius_;
    float speed_;
    float from_angle_;
    float to_angle;
  };

}

#endif //WAYPOINT_H