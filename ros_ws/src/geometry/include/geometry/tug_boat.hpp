
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

#include "tug_point.hpp"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"
#include "tug_switching_point_decision.hpp"

namespace Tug
{
  class Boat
  {
  public:
    Boat(double radius, const Point &position, Environment *environment);
    Boat(){};
    void set_top_speed(double top_speed){top_speed_ = top_speed;}
    double get_top_speed() const{return top_speed_;}
    void set_path(Polyline path){path_ = path; current_waypoint_index_=0;};
    double get_radius(){return radius_;};
    Point  get_position();
    void update_position(Point &position, bool &waypoint_updated_flag);
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
    Polyline get_path() const {return path_;}
    Point *get_current_waypoint();
    void create_switchingpoint_decision();
  private:
    double radius_;
    double top_speed_ = 1; //pixels per time unit
    Point position_;
    Environment *environment_;
    int id_=-1;
    Polyline path_;
    int current_waypoint_index_ = 0; 
    Switching_point_decision *switching_point_decision_;
    void set_switchingpoint_decision(Point &pt1, Point &pt2);

    bool go_to_next_waypoint();
  };
}

#endif //TUG_BOAT_H