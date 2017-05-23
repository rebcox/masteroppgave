
#ifndef TUG_SHIP_H
#define TUG_SHIP_H

#include "tug_point.hpp"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"
//#include "tug_switching_point_decision.hpp"

namespace Tug
{
  class Ship : public Boat
  {
  public:
    Ship(double length, double width, Environment *env) : length_(length), width_(width);

  private:
    double length;
    double width;
  };
}

#endif //TUG_SHIP_H

