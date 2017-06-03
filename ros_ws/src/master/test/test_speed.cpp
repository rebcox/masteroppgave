#include <ros/package.h>
#include <ros/ros.h>

#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "search/tug_shortest_path.hpp"
#include <chrono>


    int x_min_; int y_min_;
    int x_max_; int y_max_;
  void path_to_hole(const ClipperLib::Path &path, VisiLibity::Polygon &hole)
  {
    for(int i = 0; i < path.size(); i++)
    {
      int x = (int)path[i].X;
      int y = (int)path[i].Y;
      hole.push_back(VisiLibity::Point(x,y));
    }
  }
  void find_max_and_min_in_path(const ClipperLib::Path &path, char coordinate, int &max_val, int &min_val) 
  {
    if (!(coordinate == 'X' || coordinate == 'Y'))
    {
      return;
    }
    max_val = std::numeric_limits<int>::min();
    min_val = std::numeric_limits<int>::max();

    int current;
    for (int i = 0; i < path.size(); ++i)
    {
      if (coordinate == 'X')
      {
        current = path[i].X;        
      }
      else
      {
        current = path[i].Y;        
      }

      if (current < min_val)
      {
        min_val = current;
      }
      else if (current > max_val)
      {
        max_val = current;
      }
    }
  }

  void set_outer_boundary(const ClipperLib::Path &outer_boundary, VisiLibity::Environment &environment)
  {
    find_max_and_min_in_path(outer_boundary, 'X', x_max_, x_min_);
    find_max_and_min_in_path(outer_boundary, 'Y', y_max_, y_min_);

    VisiLibity::Polygon outer_boundary_polygon;
    path_to_hole(outer_boundary, outer_boundary_polygon);
    environment.set_outer_boundary(outer_boundary_polygon);
  }

void convert_to_visilibity_environment(const ClipperLib::Paths &paths, VisiLibity::Environment &environment)
  {
    if (paths.size() == 0)
    {
      return;
    }

    find_max_and_min_in_path(paths[0], 'X', x_max_, x_min_);
    find_max_and_min_in_path(paths[0], 'Y', y_max_, y_min_);

    int x_min_cur, x_max_cur, y_min_cur, y_max_cur;

    for (int i = 1; i < paths.size(); ++i)
    {
      find_max_and_min_in_path(paths[i], 'X', x_max_cur, x_min_cur);
      if (x_max_cur >= x_max_)
      {
        x_max_ = x_max_cur +1 ;
      }
      if (x_min_cur <= x_min_)
      {
        x_min_ = x_min_cur -1 ;
      }
      find_max_and_min_in_path(paths[i], 'Y', y_max_cur, y_min_cur);
      if (y_max_cur >= y_max_)
      {
        y_max_ = y_max_cur + 1 ;
      }
      if (y_min_cur <= y_min_)
      {
        y_min_ = y_min_cur -1 ;
      }
    }
    ClipperLib::Path path_temp;
    path_temp.push_back(ClipperLib::IntPoint(x_min_, y_min_));
    path_temp.push_back(ClipperLib::IntPoint(x_max_, y_min_));
    path_temp.push_back(ClipperLib::IntPoint(x_max_, y_max_));
    path_temp.push_back(ClipperLib::IntPoint(x_min_, y_max_));

    set_outer_boundary(path_temp, environment);

    for (int i = 1; i < paths.size(); ++i)      
    {
      VisiLibity::Polygon hole;
      path_to_hole(paths[i], hole);
      environment.add_hole(hole);
    }
  }

  bool load_from_file(ClipperLib::Paths &ppg, const std::string& filename)
  {
    //file format assumes: 
    //  1. path coordinates (x,y) are comma separated (+/- spaces) and 
    //  each coordinate is on a separate line
    //  2. each path is separated by one or more blank lines
    ppg.clear();
    std::ifstream ifs(filename);
    if (!ifs) return false;
    std::string line;
    ClipperLib::Path pg;
    while (std::getline(ifs, line))
    {
      std::stringstream ss(line);
      double X = 0.0, Y = 0.0;
      if (!(ss >> X))
      {
        //ie blank lines => flag start of next polygon 
        if (pg.size() > 0) ppg.push_back(pg);
        pg.clear();
        continue;
      }
      char c = ss.peek();  
      while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces before comma
      if (c == ',') {ss.read(&c, 1); c = ss.peek();} //gobble comma
      while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces after comma
      if (!(ss >> Y)) break; //oops!
      pg.push_back(ClipperLib::IntPoint((ClipperLib::cInt)(X),(ClipperLib::cInt)(Y)));
    }
    if (pg.size() > 0) ppg.push_back(pg);
    ifs.close();

    return true;
  }


void run_method1(std::string &filename, const Tug::Point &start, const Tug::Point &goal, int no_nodes, double epsilon)
{
  ClipperLib::Paths paths;
  VisiLibity::Environment env;
  
  load_from_file(paths, filename);
  convert_to_visilibity_environment(paths, env);
  VisiLibity::Point start_vis(start.x(), start.y());
  VisiLibity::Point goal_vis(goal.x(), goal.y());

  auto begin = std::chrono::high_resolution_clock::now();    
    VisiLibity::Polyline sol = env.shortest_path(start_vis, goal_vis, epsilon);
  auto end = std::chrono::high_resolution_clock::now();    
  auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
  std::cout << "Method 1, " << no_nodes << " nodes: " << ms << " microseconds. "<< std::endl;
}

void run_method2(Tug::Environment &env, const Tug::Point &start, const Tug::Point &goal, int no_nodes, double epsilon)
{
  Tug::Polyline shortest_path;
  auto begin = std::chrono::high_resolution_clock::now();    
    Tug::Shortest_path shortest_path_node(env, start, goal, shortest_path);
  auto end = std::chrono::high_resolution_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
  std::cout << "Method 2, " << no_nodes << " nodes: " << ms << " microseconds. "<< std::endl;
}

void run_method3(Tug::Environment &env, const Tug::Point &start, const Tug::Point &goal, int no_nodes, double epsilon)
{
  Tug::Polyline shortest_path;
  Tug::Shortest_path shortest_path_node(env);

  auto begin = std::chrono::high_resolution_clock::now();
  shortest_path_node.calculate_shortest_path(start, goal, shortest_path, env);    
  auto end = std::chrono::high_resolution_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
  std::cout << "Method 3, " << no_nodes << " nodes: " << ms << " microseconds. "<< std::endl;
}

int main(int argc, char **argv)
{
  double epsilon = 0.01;
  /*
  std::string filename10 = "/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env10.txt";
  Tug::Environment env10 = Tug::Environment(filename10, 1, epsilon);

  Tug::Point start10(145, 492, env10);
  Tug::Point goal10(684, 97, env10);

  run_method1(filename10,start10,goal10,10,epsilon);
  run_method2(env10,start10,goal10,10,epsilon);
  run_method3(env10,start10,goal10,10,epsilon);

  std::string filename20 = "/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env20.txt";
  Tug::Environment env20 = Tug::Environment(filename20, 1, epsilon);

  Tug::Point start20(145, 492, env20);
  Tug::Point goal20(684, 97, env20);

  run_method1(filename20,start20,goal20,20,epsilon);
  run_method2(env20,start20,goal20,20,epsilon);
  run_method3(env20,start20,goal20,20,epsilon);


  std::string filename50 = "/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env50.txt";
  Tug::Environment env50 = Tug::Environment(filename50, 1, epsilon);

  Tug::Point start50(10, 570, env50);
  Tug::Point goal50(868, 3, env50);

  run_method1(filename50,start50,goal50,50,epsilon);
  run_method2(env50,start50,goal50,50,epsilon);
  run_method3(env50,start50,goal50,50,epsilon);

  std::string filename100 = "/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env100.txt";
  Tug::Environment env100 = Tug::Environment(filename100, 1, epsilon);

  Tug::Point start100(10, 570, env100);
  Tug::Point goal100(868, 3, env100);

  run_method1(filename100,start100,goal100,100,epsilon);
  run_method2(env100,start100,goal100,100,epsilon);
  run_method3(env100,start100,goal100,100,epsilon);


  std::string filename50 = "/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env50.txt";
  Tug::Environment env50 = Tug::Environment(filename50, 1, epsilon);

  Tug::Point start50(10, 570, env50);
  Tug::Point goal50(868, 3, env50);

  run_method1(filename50,start50,goal50,50,epsilon);
  run_method2(env50,start50,goal50,50,epsilon);
  run_method3(env50,start50,goal50,50,epsilon);
*/
  std::string filename200 = "/home/rebecca/GITHUB/mast/ros_ws/src/master/environments/env200.txt";
  Tug::Environment env200 = Tug::Environment(filename200, 1, epsilon);

  Tug::Point start200(10, 570, env200);
  Tug::Point goal200(868, 3, env200);

  run_method1(filename200,start200,goal200,200,epsilon);
  run_method2(env200,start200,goal200,200,epsilon);
  run_method3(env200,start200,goal200,200,epsilon);
}

