#include "include/clipper.hpp"
#include "include/SVG_builder.hpp"
#include "include/visilibity.hpp" 
#include "include/tug_environment.hpp"
#include <sstream>
#include <string>
#include "include/tug_point.hpp"
#include "include/tug_all_pairs_shortest_path.hpp"
#include "include/shortest_path.h"
#include <time.h>
#include "include/tug_scheduler.hpp"

int meter_to_pixels(double scale, double meter)
{
  return round(scale*meter);
}


int main(int argc, char **argv)
{

  time_t before;
  time_t after;

  double seconds;

  ClipperLib::Paths solution;
  //double epsilon = 0.000000001;
  double epsilon = 0.001;


  Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);

  Tug::Point s1(60, 60,tug_env.visilibity_environment());
  Tug::Point f1(320,320,tug_env.visilibity_environment());

  Tug::Point s2(40, 40,tug_env.visilibity_environment());
  Tug::Point f2(40,240,tug_env.visilibity_environment());

  Tug::Point s3(200, 200,tug_env.visilibity_environment());
  Tug::Point f3(349, 1,tug_env.visilibity_environment());

  Tug::Point s4(10, 10,tug_env.visilibity_environment());
  Tug::Point f4(20, 20,tug_env.visilibity_environment());


  Tug::Point s5(58, 58, tug_env.visilibity_environment());
  Tug::Point f5(320,320,tug_env.visilibity_environment());

  std::vector<Tug::Polyline> shortest_paths;

  shortest_paths.push_back(tug_env.shortest_path(s4,f4));
  shortest_paths.push_back(tug_env.shortest_path(s1,f1));
  shortest_paths.push_back(tug_env.shortest_path(s2,f2));
  shortest_paths.push_back(tug_env.shortest_path(s3,f3));
  shortest_paths.push_back(tug_env.shortest_path(s5,f5));

  Tug::Scheduler tug_scheduler(shortest_paths, tug_env);
  tug_scheduler.print_paths(shortest_paths);

  for (int i = 0; i < shortest_paths.size(); ++i)
  {
    for (int j = 0; j < shortest_paths[i].size(); ++j)
    {
      std::cout << "id: " << shortest_paths[i][j].id() << " ";
    }
    std::cout << std::endl;
  }


  /*std::vector<int> schedule = shortest_paths[4][2].get_schedule();
  for (int k = 0; k < schedule.size(); k++)
    std::cout << schedule[k] << ", ";
*/
/*
  for (int i = 0; i < shortest_paths.size(); ++i)
  {
    for (int j = 0; j < shortest_paths[i].size(); ++j)
    {
      std::vector<int> schedule = shortest_paths[i][j].get_schedule();
      for (int k = 0; k < schedule.size(); k++)
      std::cout << schedule[k] << ", ";
    }
    std::cout << std::endl;
    
  }*/


/*
  Tug::Environment tug_env_scaled("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.5, epsilon);
  tug_env_scaled.add_constant_safety_margin(43);

  Tug::Polyline shortest_p_scaled;
  Tug::Point s_scaled(60,60,tug_env_scaled.visilibity_environment());
  Tug::Point f_scaled(320,320,tug_env_scaled.visilibity_environment());

  shortest_p_scaled = tug_env_scaled.shortest_path(s_scaled,f_scaled);

  tug_env_scaled.save_environment_as_svg("scaled_env.svg", shortest_p_scaled);

  //double epsilon = 0.000000001;

  Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);
  tug_env.add_constant_safety_margin(43);

  Tug::Polyline shortest_p;
  Tug::Point s(60, 60,tug_env.visilibity_environment());
  Tug::Point f(320,320,tug_env.visilibity_environment());

  shortest_p = tug_env.shortest_path(s,f);

  tug_env.save_environment_as_svg("not_scaled_env.svg", shortest_p);

  */


/*
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);
  //Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);


  Tug::Polyline my_shortest_path_after_safety;

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

  //Tug::Environment asps_env("/home/rebecca/GITHUB/mast/oppg/environments/apsp_env.txt", 1.0, epsilon);
  Tug::Environment asps_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);

 // Tug::Environment asps_env("/Users/rebeccacox/GitHub/mast/oppg/environments/apsp_env.txt", 1.0, epsilon);

  Tug::All_pairs_shortest_path apsp(asps_env);

  Tug::Shortest_path sp("/home/rebecca/GITHUB/mast/oppg/build/all_pairs_shortest_path.txt");

  Tug::Polyline shortest_path_calculated;

  sp.calculate_shortest_path(1, 13, shortest_path_calculated, tug_env);

  std::cout << "Shortest path from 1 to 13: ";
    for (int i = 0; i < shortest_path_calculated.size(); ++i)
    {
      std::cout << shortest_path_calculated[i] << "  ";
    }
    std::cout << std::endl;

*/
    /*
  double env_pixel_width = 1076;
  double env_width = 2000;
  double scale = env_pixel_width/env_width;


  before = time(NULL);

  Tug::Environment big_tug_env("/home/rebecca/GITHUB/mast/oppg/environments/peterhead.txt", 1.0, epsilon);
  big_tug_env.add_constant_safety_margin(meter_to_pixels(scale, 8));
  Tug::Polyline shortest_p_for_big_tug;
  Tug::Point start_for_big_tug(210,170,big_tug_env.visilibity_environment());
  Tug::Point finish_for_big_tug(1020,270,big_tug_env.visilibity_environment());

  shortest_p_for_big_tug = big_tug_env.shortest_path(start_for_big_tug,finish_for_big_tug);
  after = time(NULL);
  seconds = difftime(after, before);
  std::cout << "IT TOOK " << seconds << " seconds" << std::endl;

  std::stringstream shortest_p_for_big_tug_path_print;
  for (int i = 0; i < shortest_p_for_big_tug.size(); ++i)
  {
    shortest_p_for_big_tug_path_print << shortest_p_for_big_tug[i];
    if (i < shortest_p_for_big_tug.size()-1)
    {
      shortest_p_for_big_tug_path_print << " - ";
    }
  }
  std::cout << "Shortest path: " << shortest_p_for_big_tug_path_print.str().c_str() << std::endl;

  big_tug_env.save_environment_as_svg("big_test.svg", shortest_p_for_big_tug);
*/

  return 0;
}