#ifndef TUG_ASSIGN_PATHS
#define TUG_ASSIGN_PATHS

#include "tug_polyline.hpp"
#include "tug_environment.hpp"
#include "tug_boat.hpp"

#include "munkres/munkres.h"

namespace Tug
{
  class Assign_paths
  {
  public:
    bool assign_on_combined_shortest_path(std::vector<Boat> &tugs, 
                                          const std::vector<Point> &finish_points, 
                                          Environment &environment);

  private:
    double eucledian_distance(const Point &point1, const Point &point2);
    void assign_goal(Boat &tug, Point &goal);


  };
}

#endif //TUG_ASSIGN_PATHS