#include "include/clipper.hpp"
#include "include/SVG_builder.hpp"
#include "include/visilibity.hpp" 
#include "include/tug_environment.hpp"
#include <sstream>
#include <string>
#include "include/tug_point.hpp"

int main(int argc, char **argv)
{
  //VisiLibity::Point start(1.0, 235);
  //VisiLibity::Point finish(400, 400);
  //VisiLibity::Point finish(235, 185);
 // VisiLibity::Point finish(220, 170);
 // VisiLibity::Point finish(40, 10);

  //VisiLibity::Point finish(150.0, 150.0);

  //Tug::Point start(30, 180);

//  Tug::Point finish(347, 230);
  
  Tug::Point start(0, 320);

  Tug::Point finish(347, 180);

 //Tug::Point finish(350, 350);



  ClipperLib::Paths solution;
  //double epsilon = 0.000000001;
  double epsilon = 0.001;

  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);

  VisiLibity::Polyline my_shortest_path_after_safety;

  tug_environment.save_environment_as_svg("sol_without.svg", my_shortest_path_after_safety);

  tug_environment.add_constant_safety_margin(40);

  
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
  return 0;
}