
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

#include "tug_point.hpp"
#include "tug_polyline.hpp"

namespace Tug
{
  class Boat
  {
  public:
    Boat(double radius, const Point &position);
    void set_path(Polyline path){path_ = path;};
    double get_radius(){return radius_;};
    Point  get_position();
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
    Polyline get_path() const {return path_;};
  private:
    double radius_;
    Point position_;
    int id_=-1;
    Polyline path_;
  };
}

#endif //TUG_BOAT_H