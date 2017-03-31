
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

#include "tug_point.hpp"

namespace Tug
{
  class Boat
  {
  public:
    Boat(double radius, const Point &position);
    double get_radius(){return radius_;};
    Point  get_position();
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
  private:
    double radius_;
    Point position_;
    int id_=-1;
  };
}

#endif //TUG_BOAT_H