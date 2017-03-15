#include "include/tug_environment.hpp"
#include <sstream>
#include <limits>
#include "include/shortest_path.h"


namespace Tug
{
  Environment::Environment(const std::string& filename, double scale, double epsilon)
  {
    epsilon_ = epsilon;
    ClipperLib::Paths temp_paths;
    bool ok = load_from_file(temp_paths, filename, scale);
    clip_against_outer_boundary(temp_paths, paths_);
    //std::cout << "Number of objects: " << paths_.size() << std::endl;
    if(!ok)
    {
      std::cout << "Could not read environment file" << std::endl;
    }
    convert_to_visilibity_environment(paths_, visilibity_environment_);

    if (!visilibity_environment_.is_valid(epsilon))
    {
      std::cout << "Environment is not valid" << std::endl;
    }
    visibility_graph_ = VisiLibity::Visibility_Graph(visilibity_environment_, epsilon);
    
    update_tug_point_list(paths_, points_in_environment_);

    /*for (int i = 0; i < points_in_environment_.size(); ++i)
    {
      std::cout << "point: " << points_in_environment_[i] << std::endl;
      std::cout << "neighbor 1: " << *points_in_environment_[i].get_neighbor1() << std::endl;
      std::cout << "neighbor 2: " << *points_in_environment_[i].get_neighbor2() << std::endl;
      std::cout << "\n";
    }*/

  }
  
  const std::vector<Point> &Environment::points() const
  {
    if (environment_has_safety_margin)
    {
      return points_in_environment_with_safety_margin_;
    }
    else
    {
      return points_in_environment_;
    }
  }
  void Environment::mark_point_as_on_boundary(unsigned k)
  {
    if (environment_has_safety_margin)
    {
      points_in_environment_with_safety_margin_[k].is_on_outer_boundary = true;
    }
    else
    {
      points_in_environment_[k].is_on_outer_boundary = true;
    }
  }

  const Point &Environment::operator() (unsigned k) const
  {
    if (environment_has_safety_margin)
    {
      return points_in_environment_with_safety_margin_.at(k);
    }
    else
    {
      return points_in_environment_.at(k);
    }
  }


  unsigned Environment::n() const
  {
    if (environment_has_safety_margin)
    {
      return visilibity_environment_with_safety_margin_.n();
    }
    else
    {
      return visilibity_environment_.n();
    }
  }

  const VisiLibity::Visibility_Graph &Environment::visibility_graph() const
  {
    if (environment_has_safety_margin)
    {
      return visibility_graph_with_safety_margin_;
    }
    else
    {
      return visibility_graph_;
    }
  }

  const VisiLibity::Environment &Environment::visilibity_environment() const
  {
    if (environment_has_safety_margin)
    {
      return visilibity_environment_with_safety_margin_;
    }
    else
    {
      return visilibity_environment_;
    }
  }
      
  void Environment::clip_against_outer_boundary(ClipperLib::Paths &paths_in, ClipperLib::Paths &paths_out)
  {
    ClipperLib::Clipper clipper;
    clipper.AddPath(paths_in[0], ClipperLib::ptClip, true);

    for (int i = 1; i < paths_in.size(); ++i)
    {
      clipper.AddPath(paths_in[i], ClipperLib::ptSubject, true);
    }
    ClipperLib::Paths temp_paths;
    clipper.Execute(ClipperLib::ctIntersection, temp_paths, ClipperLib::pftEvenOdd);
    paths_out.push_back(paths_in[0]);

    for (int i = 0; i < temp_paths.size(); ++i)
    {
      reverse_path(temp_paths[i]);
      paths_out.push_back(temp_paths[i]);
    }
    
  }

  void Environment::add_constant_safety_margin(double margin)
  {
    ClipperLib::ClipperOffset co;
    //Add safety margin
    for (int i = 1; i < paths_.size(); ++i)
    {
      co.AddPath(paths_[i], ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    }
    ClipperLib::Paths tempPaths;
    co.Execute(tempPaths, margin);

    //clip against outer boundary
    ClipperLib::Clipper clipper;
    clipper.AddPath(paths_[0], ClipperLib::ptClip, true);
    for (int i = 0; i < tempPaths.size(); ++i)
    {
      clipper.AddPath(tempPaths[i], ClipperLib::ptSubject, true);
    }
    ClipperLib::Paths tempPaths2;
    clipper.Execute(ClipperLib::ctIntersection, tempPaths2, ClipperLib::pftEvenOdd);
    
    //Outer boundary
    paths_with_safety_margin_.push_back(paths_[0]);
    
    for (int i = 0; i < tempPaths2.size(); ++i)
    {
      reverse_path(tempPaths2[i]);
      paths_with_safety_margin_.push_back(tempPaths2[i]);
    }

    convert_to_visilibity_environment(paths_with_safety_margin_, visilibity_environment_with_safety_margin_);
    
    visilibity_environment_with_safety_margin_.is_valid(epsilon_);

    visibility_graph_with_safety_margin_ = VisiLibity::Visibility_Graph(visilibity_environment_with_safety_margin_, epsilon_);

    environment_has_safety_margin = true;
    update_tug_point_list(paths_with_safety_margin_, points_in_environment_with_safety_margin_);
  }

  bool Environment::point_is_within_outer_boundary(const ClipperLib::IntPoint point)
  {
    if (point.X >= x_max-1 || point.X <= x_min+1 || point.Y <= y_min+1 || point.Y >= y_max-1) 
    {
      return false;
    }
    return true;
  }

  void Environment::reverse_path(ClipperLib::Path &path)
  {
    std::reverse(path.begin(), path.end());
  }

  void Environment::path_to_hole(const ClipperLib::Path &path, VisiLibity::Polygon &hole)
  {
    for(int i = 0; i < path.size(); i++)
    {
      int x = (int)path[i].X;
      int y = (int)path[i].Y;
      hole.push_back(VisiLibity::Point(x,y));
    }
  }

  void Environment::convert_to_visilibity_environment(const ClipperLib::Paths &paths, VisiLibity::Environment &environment)
  {
    find_max_and_min_in_path(paths[0], 'X', x_max, x_min);
    find_max_and_min_in_path(paths[0], 'Y', y_max, y_min);

    int x_min_cur, x_max_cur, y_min_cur, y_max_cur;

    for (int i = 1; i < paths.size(); ++i)
    {
      find_max_and_min_in_path(paths[i], 'X', x_max_cur, x_min_cur);
      if (x_max_cur >= x_max)
      {
        x_max = x_max_cur +1 ;
      }
      if (x_min_cur <= x_min)
      {
        x_min = x_min_cur -1 ;
      }
      find_max_and_min_in_path(paths[i], 'Y', y_max_cur, y_min_cur);
      if (y_max_cur >= y_max)
      {
        y_max = y_max_cur + 1 ;
      }
      if (y_min_cur <= y_min)
      {
        y_min = y_min_cur -1 ;
      }
    }
    ClipperLib::Path path_temp;
    path_temp.push_back(ClipperLib::IntPoint(x_min, y_min));
    path_temp.push_back(ClipperLib::IntPoint(x_max, y_min));
    path_temp.push_back(ClipperLib::IntPoint(x_max, y_max));
    path_temp.push_back(ClipperLib::IntPoint(x_min, y_max));

    set_outer_boundary(path_temp, environment);

    for (int i = 1; i < paths.size(); ++i)      
    {
      VisiLibity::Polygon hole;
      path_to_hole(paths[i], hole);
      environment.add_hole(hole);
    }
  }

  void Environment::update_tug_point_list(const ClipperLib::Paths &paths, std::vector<Point> &tug_points)
  {
    int k = 0;
    tug_points.clear();

    int counter = 0;
    for (int i = 0; i < paths.size(); ++i)
    {
      for (int j = 0; j < paths[i].size(); ++j)
      {
        //Point(paths[i][j], paths[i][j-1], paths[j+1]);
        //tug_points.push_back(Point(paths[i][j]));

        if (i != 0) //not outer boundary
        {
          //pt.set_point_id(++counter);
          tug_points.push_back(Point(paths[i][j], visilibity_environment(), ++counter));

        }
        else
        {
          tug_points.push_back(Point(paths[i][j], visilibity_environment()));
        }

        //std::cout << tug_points.back() << std::endl;
      }
      int path_size = paths[i].size();

      tug_points[k].set_neighbor1(tug_points[k+path_size-1]);
      tug_points[k].set_neighbor2(tug_points[k+1]);
      
      for (int j = 1; j < path_size-1; ++j)
      {
        tug_points[k+j].set_neighbor1(tug_points[k+j-1]);
        tug_points[k+j].set_neighbor2(tug_points[k+j+1]);
      }  

      tug_points[k+path_size-1].set_neighbor1(tug_points[k+path_size-2]);
      tug_points[k+path_size-1].set_neighbor2(tug_points[k]);

      k+=path_size;
    }
    mark_points_touching_outer_boundary();
  }

  Polyline Environment::shortest_path(const Point &start, const Point &finish) //, double epsilon)
  {
    Polyline shortest_path;
    std::vector<bool> points_to_remove;

    Shortest_path((*this), start,finish,shortest_path);
    return shortest_path;
  }

  void Environment::set_outer_boundary(const ClipperLib::Path &outer_boundary, VisiLibity::Environment &environment)
  {
    find_max_and_min_in_path(outer_boundary, 'X', x_max, x_min);
    find_max_and_min_in_path(outer_boundary, 'Y', y_max, y_min);

    VisiLibity::Polygon outer_boundary_polygon;
    path_to_hole(outer_boundary, outer_boundary_polygon);
    environment.set_outer_boundary(outer_boundary_polygon);
  }

  void Environment::mark_points_touching_outer_boundary() 
  {
    std::vector<bool> out;
    for (int i = 0; i < n(); ++i)
    {
      if (environment_has_safety_margin and point_is_on_outer_boundary(points_in_environment_with_safety_margin_[i]))
      {
        points_in_environment_with_safety_margin_[i].is_on_outer_boundary = true;
      }
      else if (!environment_has_safety_margin and point_is_on_outer_boundary(points_in_environment_[i]))
      {
        points_in_environment_[i].is_on_outer_boundary = true;
      }
    }
  }

  bool Environment::point_is_on_outer_boundary(const VisiLibity::Point &point)
  {
    //outer boundary is 1 unit smaller
    if (point.x() == x_max-1 || point.x() == x_min+1 || point.y() == y_min+1 || point.y() == y_max-1)
    {
      return true;
    }
    return false;
  }

  bool Environment::load_from_file(ClipperLib::Paths &ppg, const std::string& filename, double scale)
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
      pg.push_back(ClipperLib::IntPoint((ClipperLib::cInt)(X * scale),(ClipperLib::cInt)(Y * scale)));
    }
    if (pg.size() > 0) ppg.push_back(pg);
    ifs.close();

    return true; //is_valid_environment(ppg);
  }


  void Environment::find_max_and_min_in_path(const ClipperLib::Path &path, char coordinate, int &max_val, int &min_val) //, ClipperLib::cInt &y_max, ClipperLib::cInt &y_min)
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

  void Environment::save_environment_as_svg(const std::string filename)
  {
    Polyline dummy;
    save_environment_as_svg(filename, dummy);
  }

  void Environment::save_environment_as_svg(const std::string filename, const Polyline &shortest_path)
  {
    SVGBuilder svg;    
    svg.style.brushClr = 0x129C0000;
    svg.style.penClr = 0xCCFFA07A;
    svg.style.pft = ClipperLib::pftEvenOdd;

    if (environment_has_safety_margin)
    {
      svg.AddPaths(paths_with_safety_margin_);
      svg.AddPaths(paths_);
    }
    else
    {
      svg.AddPaths(paths_);
    }
    if (shortest_path.size()>0)
    {
      svg.AddPolyline(shortest_path);
    }
    svg.SaveToFile(filename, 1,0);
  }
}
