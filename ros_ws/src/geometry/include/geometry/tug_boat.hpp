
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

#include "tug_point.hpp"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"
#include <memory>

namespace Tug
{
  class Boat
  {
  public:
    Point get_position();
    void update_position(Point &position);
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
    void set_path(const Polyline &path);

  private:
    Point position_ = Point(-1,-1,-1);
    int id_=-1;
  };
}

#endif //TUG_BOAT_H

