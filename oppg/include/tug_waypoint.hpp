#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "visilibity.hpp"
#include "tug_point.hpp"
#include "tug_environment.hpp"

namespace Tug
{
  class Waypoint : public Point
  {
  public:
   // Waypoint(const Point &point, float range, const Environment &environment, std::vector<Waypoint> &waypoints, int id);
    Waypoint(Point &point, int point_id, int range, std::vector<Waypoint> &waypoints);
    void set_points_within_range(float range, std::vector<Waypoint> &waypoints);
    float eucledian_distance(const Point &point1, const Point &point2);
    bool is_within_safety_region();
    float get_radius(){return radius_;};
    float get_speed(){return speed_;};
    void possible_angle_range(VisiLibity::Angle &from_angle, VisiLibity::Angle &to_angle);
    Point get_position(){return *this;}
    void add_point_within_range(Waypoint *waypoint){points_within_range_.push_back(waypoint);}
    Waypoint* point_within_range(int i);
    int n_points_within_range(){return points_within_range_.size();}
    //int id(){return id_;}
    bool is_available(int t);
    void set_tug(int id, int t);
    std::vector<int> &get_schedule(){return schedule_;};
  private:
    std::vector<Waypoint*> points_within_range_;
    std::vector<int> schedule_;

    float radius_ = 0.0;
    float speed_ = 0.0;
    float from_angle_;
    float to_angle;
  };

}

#endif //WAYPOINT_H