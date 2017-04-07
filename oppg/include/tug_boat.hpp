
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

#include "tug_point.hpp"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"

namespace Tug
{
  class Boat
  {
  public:
    Boat(double radius, const Point &position, Environment *environment);
    void set_top_speed(double top_speed){top_speed_ = top_speed;}
    double get_top_speed() const{return top_speed_;}
    void set_path(Polyline path){path_ = path;};
    void set_path(const Point &finish);
    double get_radius(){return radius_;};
    Point  get_position();
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
    Polyline get_path() const {return path_;};
  private:
    double radius_;
    double top_speed_ = 1; //pixels per time unit
    Point position_;
    Environment *environment_;
    int id_=-1;
    Polyline path_;
  };
}

#endif //TUG_BOAT_H