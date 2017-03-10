#include "include/clipper.hpp"
#include "include/SVG_builder.hpp"
#include "include/visilibity.hpp" 
#include "include/tug_environment.hpp"
#include <sstream>
#include <string>
#include "include/tug_point.hpp"
#include "include/tug_all_pairs_shortest_path.hpp"


int main(int argc, char **argv)
{

 // Tug::Point start(1, 320);
 // Tug::Point finish(325, 180);


  ClipperLib::Paths solution;
  //double epsilon = 0.000000001;
  double epsilon = 0.001;

  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);

  VisiLibity::Polyline my_shortest_path_after_safety;

  tug_environment.save_environment_as_svg("sol_without.svg", my_shortest_path_after_safety);

  tug_environment.add_constant_safety_margin(43);


  Tug::Point start(30, 180, tug_environment.visilibity_environment());
  Tug::Point finish(347, 230, tug_environment.visilibity_environment());


  
  my_shortest_path_after_safety = tug_environment.shortest_path(start,finish); //,epsilon);

  tug_environment.save_environment_as_svg("sol_w_shortest_path.svg", my_shortest_path_after_safety);

 // ROS_INFO("length of path: %f", my_shortest_path_after_safety.length());
  std::stringstream shortest_path_print;
  for (int i = 0; i < my_shortest_path_after_safety.size(); ++i)
  {
    shortest_path_print << my_shortest_path_after_safety[i];
    if (i < my_shortest_path_after_safety.size()-1)
    {
      shortest_path_print << " - ";
    }
  }
  std::cout << "Shortest path: " << shortest_path_print.str().c_str() << std::endl;


  Tug::Environment asps_env("/home/rebecca/GITHUB/mast/oppg/environments/apsp_env.txt", 1.0, epsilon);

  Tug::All_pairs_shortest_path apsp(asps_env);

  return 0;
}