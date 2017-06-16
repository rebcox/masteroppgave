#ifndef TUG_ASSIGN_PATHS
#define TUG_ASSIGN_PATHS

#include "external/munkres.h"
#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"

namespace Tug
{
  class Assign_paths
  {
  public:
    Assign_paths(Environment environment)
    {
      shortest_path_node_ptr = std::make_shared<Shortest_path>(environment);
    }

    bool assign_on_combined_shortest_path(std::map<int, Boat> &tugs, 
                                          const std::vector<Point> &poal_pts, 
                                          Environment &environment);
    bool assign_on_combined_shortest_path(std::vector<Boat> &tugs, 
                                          std::map<int, Point> &poal_pts, 
                                          Environment &environment);
  private:
    double euclidean_distance(const Point &point1, const Point &point2);

    bool assign(std::vector<Boat> &tugs, 
                const std::vector<Point> &poal_pts, 
                Environment &environment);
    std::shared_ptr<Shortest_path> shortest_path_node_ptr;

  };
}

#endif //TUG_ASSIGN_PATHS