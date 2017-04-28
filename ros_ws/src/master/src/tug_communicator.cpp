#include "tug_communicator.hpp"

namespace Tug
{
  Communicator::Communicator(Boat &tug)
  {
    path_ = tug.get_path();
  }

  Waypoint* Communicator::waypoint(const Boat &tug)
  {

  }

  void Communicator::update_waypoint(Waypoint &prev_pt, Waypoint &new_pt)
  {
   // switching_point_decision_ = Switching_point_decision(prev_pt, new_pt);

  }

}