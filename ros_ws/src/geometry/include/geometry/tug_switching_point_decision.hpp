#ifndef SWICHING_POINT_DECISION
#define SWICHING_POINT_DECISION

#include "tug_environment.hpp"
#include "tug_waypoint.hpp"

namespace Tug
{
  class Switching_point_decision
  {
  public:
    Switching_point_decision(Point &pt1, Point &pt2);
    bool accept(const Point &position, double radius);

  private:
    double angle_;
    double distance_between_points_;
    Point start_pt_;

    double along_track_distance(const Point &position, const Point &pt1, double angle);
    double angle(Point &pt1, Point &pt2);
  };
}

#endif //SWICHING_POINT_DECISION