#ifndef TUG_ASSIGN_PATHS
#define TUG_ASSIGN_PATHS

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"

#include "external/munkres.h"

namespace Tug
{
  class Assign_paths
  {
  public:
  	/*Assigner(std::vector<Boat> &tugs, 
	          const std::vector<Point> &finish_points, 
	          Environment &environment)
  	{
  	  assign_on_combined_shortest_path(tugs,finish_points,environment);
  	}*/

    bool assign_on_combined_shortest_path(std::map<int, Boat> &tugs, 
                                          const std::vector<Point> &finish_points, 
                                          Environment &environment);
  private:
    double euclidean_distance(const Point &point1, const Point &point2);
        bool assign_on_combined_shortest_path(std::vector<Boat> &tugs, 
                                          const std::vector<Point> &finish_points, 
                                          Environment &environment);
  };
}

#endif //TUG_ASSIGN_PATHS