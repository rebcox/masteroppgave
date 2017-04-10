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
#include "include/hungarian.hpp"
#include "include/munkres/munkres.h"
#include "include/tug_assign_paths.hpp"

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


  Tug::Environment tug_env("/Users/rebeccacox/GitHub/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);
 // Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);

  std::vector<Tug::Point> start_points;
  start_points.push_back(Tug::Point(60, 60, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(40, 40, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(250, 200, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(10, 10, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(50, 50, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(56, 30, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(15, 15, tug_env.visilibity_environment()));
  start_points.push_back(Tug::Point(280, 200, tug_env.visilibity_environment()));


  std::vector<Tug::Point> finish_points;
  finish_points.push_back(Tug::Point(318, 320, tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(40, 240, tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(349, 1, tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(20, 20, tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(308, 322, tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(325 , 340, tug_env.visilibity_environment()));


  std::vector<Tug::Boat> tugs;

  for (int i = 0; i < start_points.size(); ++i)
  {
    Tug::Boat tug(7.0, start_points[i], &tug_env); 
    tug.set_id(i+1);
    tug.set_top_speed(1);
    tugs.push_back(tug);
  }

  Tug::Assign_paths assigner;
  bool ok = assigner.assign_on_combined_shortest_path(tugs, finish_points, tug_env);
  if (!ok) return -1;

  std::vector<Tug::Polyline> shortest_paths;
  for (int i = 0; i < tugs.size(); ++i)
  {
    shortest_paths.push_back(tugs[i].get_path());
  }

  tug_env.save_environment_as_svg("tugs.svg", shortest_paths);

  Tug::Scheduler tug_scheduler(tugs, tug_env);

  tug_scheduler.print_schedule();
  tug_scheduler.print_paths(tugs);



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