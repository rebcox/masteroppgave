#include "include/clipper.hpp"
#include "include/SVG_builder.hpp"
#include "include/visilibity.hpp" 
#include "include/tug_environment.hpp"
#include <sstream>
#include <string>
#include "include/tug_point.hpp"
#include "include/tug_all_pairs_shortest_path.hpp"
#include "include/shortest_path.h"


int meter_to_pixels(double scale, double meter)
{
  return round(scale*meter);
}


int main(int argc, char **argv)
{
  double epsilon = 0.001;
  double env_pixel_width = 1076;
  double env_width = 2000;
  double scale = env_pixel_width/env_width;

  Tug::Environment peterhead_env("/home/rebecca/GITHUB/mast/oppg/environments/peterhead.txt", 1.0, epsilon);

  Tug::Point start_peterhead(186,170,peterhead_env.visilibity_environment());
  Tug::Point finish_peterhead(1020,270,peterhead_env.visilibity_environment());
  
  Tug::Polyline shortest_path_peterhead1;
  shortest_path_peterhead1 = peterhead_env.shortest_path(start_peterhead,finish_peterhead);
  peterhead_env.save_environment_as_svg("peterhead1.svg", shortest_path_peterhead1);


  peterhead_env.add_constant_safety_margin(meter_to_pixels(scale, 12));
  Tug::Point start_peterhead2(186,170,peterhead_env.visilibity_environment());
  Tug::Point finish_peterhead2(1020,270,peterhead_env.visilibity_environment());
  
  Tug::Polyline shortest_path_peterhead2;
  shortest_path_peterhead2 = peterhead_env.shortest_path(start_peterhead2,finish_peterhead2);
  peterhead_env.save_environment_as_svg("peterhead2.svg", shortest_path_peterhead2);



  return 0;
}