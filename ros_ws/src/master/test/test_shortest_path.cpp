#include <ros/package.h>
#include <ros/ros.h>

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"

int main(int argc, char **argv)
{
  ClipperLib::Paths solution;
  double epsilon = 0.001;


 // Tug::Environment tug_env("/Users/rebeccacox/GitHub/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);
  Tug::Environment tug_env("/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env100.txt", 1.0, epsilon);
  tug_env.add_constant_safety_margin(5);
  Tug::Point start(10, 570, tug_env);
  Tug::Point finish(450, 3, tug_env);

  Tug::Shortest_path sp_node(tug_env);
  Tug::Polyline shortest_path;
  bool ok = sp_node.calculate_shortest_path(start, finish, shortest_path, tug_env);


  tug_env.save_environment_as_svg("dense_env_w_safety.svg", shortest_path);


  return 0;
}