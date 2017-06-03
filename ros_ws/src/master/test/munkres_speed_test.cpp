#include <ros/package.h>
#include <ros/ros.h>

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"
#include "coordination/tug_assign_paths.hpp"
#include <chrono>

int main(int argc, char **argv)
{
  ClipperLib::Paths solution;
  double epsilon = 0.001;


 // Tug::Environment tug_env("/Users/rebeccacox/GitHub/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);
  Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, epsilon);

  std::vector<Tug::Point> start_points;
  start_points.push_back(Tug::Point(60, 60, tug_env));
  start_points.push_back(Tug::Point(40, 40, tug_env));
  start_points.push_back(Tug::Point(250, 200, tug_env));
  start_points.push_back(Tug::Point(10, 10, tug_env));
  start_points.push_back(Tug::Point(50, 50, tug_env));
  start_points.push_back(Tug::Point(56, 30, tug_env));
  start_points.push_back(Tug::Point(15, 15, tug_env));
  start_points.push_back(Tug::Point(280, 200, tug_env));
  start_points.push_back(Tug::Point(330 ,320, tug_env));
  start_points.push_back(Tug::Point(5, 318, tug_env));


  std::vector<Tug::Point> finish_points;
  finish_points.push_back(Tug::Point(318, 320, tug_env));
  finish_points.push_back(Tug::Point(40, 240, tug_env));
  finish_points.push_back(Tug::Point(349, 1, tug_env));
  finish_points.push_back(Tug::Point(20, 20, tug_env));
  finish_points.push_back(Tug::Point(308, 322, tug_env));
  finish_points.push_back(Tug::Point(325 , 340, tug_env));
  finish_points.push_back(Tug::Point(250, 150, tug_env));
  finish_points.push_back(Tug::Point(310, 324, tug_env));
  finish_points.push_back(Tug::Point(170, 200, tug_env));
  finish_points.push_back(Tug::Point(50, 290, tug_env));



  std::vector<Tug::Boat> tugs;

  for (int i = 0; i < start_points.size(); ++i)
  {
    Tug::Boat tug(7.0, start_points[i], &tug_env); 
    tug.set_id(i+1);
    tug.set_top_speed(1);
    tugs.push_back(tug);
  }

  Tug::Assign_paths assigner;
  auto t1 = std::chrono::high_resolution_clock::now();
  bool ok = assigner.assign(tugs, finish_points, tug_env);
  auto t2 = std::chrono::high_resolution_clock::now();

  if (!ok) return -1;

  ROS_INFO("test for %lu tugs function took %ld milliseconds", tugs.size(), std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() );

  std::vector<Tug::Polyline> shortest_paths;
  for (int i = 0; i < tugs.size(); ++i)
  {
    shortest_paths.push_back(tugs[i].get_path());
  }

  tug_env.save_environment_as_svg("speed_test.svg", shortest_paths);


  return 0;
}