#ifndef SHORTEST_PATH_H
#define SHORTEST_PATH_H

#include "visilibity.hpp"
#include "tug_environment.hpp"
#include "waypoint.h"
#include "clipper.hpp"
#include "tug_point.hpp"
#include "tug_a_star_search.hpp"
#include "math.h"
#include <limits>

namespace Tug
{
  class Shortest_path
  {
  public:
    Shortest_path(Tug::Environment &environment, const Point &start, 
                  const Point &finish, Polyline &shortest_path);
    Shortest_path(const std::string &all_pairs_shortest_path);

    //void calculate_shortest_path(const Point &start, const Point &finish, Polyline &shortest_path, Tug::Environment &environment);
    void calculate_shortest_path(int start_id, int finish_id, Polyline &shortest_path, Environment &environment);

    std::vector<Waypoint> get_waypoints();

  private:
   // Polyline shortest_path_;
    std::vector<std::vector<int>> apsp_;

    std::vector<Waypoint> waypoints_;
    bool read_file(const std::string &filename);
    void set_waypoints(Polyline &shortest_path);
    bool is_valid_start_and_end_points(const Point &start, const Point &finish, 
                                      const Tug::Environment &environment);
    int point_within_safety_margin(const Point &point, const Tug::Environment &environment);
    Point point_closest_to_line_segment(const Point &point, const VisiLibity::Line_Segment &line);
    Point calculate_point_on_boundary(const Point &point, const VisiLibity::Polygon &hole, const Tug::Environment &env);
    //Point calculate_point_on_boundary(const Point &point, const VisiLibity::Polygon &hole);

   // std::vector<bool> remove_points_touching_outer_boundary(VisiLibity::Environment &environment);
    //std::vector<bool> points_touching_outer_boundary(Tug::Environment &environment);
    bool point_is_on_outer_boundary(const Point &point, const Tug::Environment &env);
    bool point_is_on_outer_boundary(const VisiLibity::Point &point, const Tug::Environment &env);

  };

}

#endif //SHORTEST_PATH_H