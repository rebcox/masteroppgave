#include "geometry/tug_waypoint.hpp"
#include "geometry/tug_boat.hpp"
#include "geometry/tug_polyline.hpp"

#include "tug_switching_point_decision.hpp"

namespace Tug
{
  class Communicator
  {
  public:
    Communicator(Boat &tug);
    Waypoint* waypoint(const Boat &tug);
  private:
   // Switching_point_decision switching_point_decision_;
    Polyline path_;
    void update_waypoint(Waypoint &prev_pt, Waypoint &new_pt);
 };

}