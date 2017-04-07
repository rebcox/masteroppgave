
#include "include/waypoint.h"
#include <cassert>

namespace Tug
{ 
  /*Waypoint::Waypoint(const Point &point, float range, const Environment &environment, std::vector<Waypoint> &waypoints, int id)
  {
   // set_x(point.x());
    //set_y(point.y());
    id_ = id;
  }*/
  Waypoint::Waypoint(Point &point, int point_id, int range, std::vector<Waypoint> &waypoints) 
   :Tug::Point(point.x(), point.y(), point_id)
  {
     
    set_points_within_range(range,waypoints);
  }


  void Waypoint::set_points_within_range(float range, std::vector<Waypoint> &waypoints)
  {
    for (int i = 0; i < waypoints.size(); ++i)
    {
      if (eucledian_distance(*this, waypoints[i].get_position()) <= range)
      {
        points_within_range_.push_back(&waypoints[i]);
        waypoints[i].add_point_within_range(this);
      }
    }
  }

  Waypoint* Waypoint::point_within_range(int i)
  {
    assert(points_within_range_.size()<i);
    return points_within_range_[i];
  }

  bool Waypoint::is_available(int t)
  {
    if (t >= schedule_.size() || schedule_[t] == 0)
    {
      return true;
    }
    return false;
  }

  void Waypoint::set_tug(int id, int t)
  {
    if (t >= schedule_.size())
    {
      int no_extra_elements = t - schedule_.size() + 1;
      std::vector<int> zeros(no_extra_elements, 0);
      schedule_.insert(schedule_.end(), zeros.begin(), zeros.end());
    }
    schedule_[t] = id;
  }


  float Waypoint::eucledian_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }

  bool Waypoint::is_within_safety_region()
  {
    return true;
  }

  void Waypoint::possible_angle_range(VisiLibity::Angle &from_angle, VisiLibity::Angle &to_angle)
  {

  }


}
