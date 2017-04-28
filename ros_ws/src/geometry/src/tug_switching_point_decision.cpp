
#include "tug_switching_point_decision.hpp"
#include <cmath>

namespace Tug
{
  Switching_point_decision::Switching_point_decision(Point &pt1, Point &pt2)
  {
    angle_ = angle(pt1, pt2);
    start_pt_ = pt1;
    distance_between_points_ = sqrt(pow(pt1.x() - pt2.x(), 2) + pow(pt1.y() - pt2.y(), 2));
  }

  double Switching_point_decision::along_track_distance(const Point &position, const Point &start_pt, double angle)
  {
    return (position.x()-start_pt.x())*std::cos(angle) + (position.y()-start_pt.y())*std::sin(angle);
  }

  double Switching_point_decision::angle(Point &pt1, Point &pt2)
  {
    return std::atan2(pt2.y() - pt1.y(), pt2.x() - pt1.x());
  }

  bool Switching_point_decision::accept(const Point &position, double radius)
  {
    return (distance_between_points_ - along_track_distance(position, start_pt_, angle_) <= radius);
  }
}
