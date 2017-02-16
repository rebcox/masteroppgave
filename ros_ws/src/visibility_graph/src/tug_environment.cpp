#include "tug_environment.h"
#include <sstream>
#include <limits>

namespace Tug
{
  Environment::Environment(const std::string& filename, double scale)
  {
    bool ok = load_from_file(paths_, filename, scale);
    std::cout << "Number of objects: " << paths_.size() << std::endl;
    if(!ok)
    {
      std::cout << "Could not read environment file or environment not valid" << std::endl;
    }
  }

  void Environment::add_constant_safety_margin(double margin) //, ClipperLib::Paths &solution)
  {
    ClipperLib::ClipperOffset co;
    for (int i = 0; i < paths_.size(); ++i)
    {
      co.AddPath(paths_[i], ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    }
    co.Execute(paths_with_safety_margin_, margin);
  }

  void Environment::path_to_hole(const ClipperLib::Path &path, VisiLibity::Polygon &hole)
  {
    //for (int i = 0; i < path.size(); ++i)
      for(int i = path.size()-1; i >=0; i--)
    {
      //CLipperLib::cInt x_cInt = path[i].X;
      int x = (int)path[i].X;
      int y = (int)path[i].Y;

      hole.push_back(VisiLibity::Point(x,y));
    }
  }

  void Environment::convert_to_visilibity_environment(const ClipperLib::Paths &paths)
  {
    //visilibity_environment_.set_outer_boundary(outer_boundary_);

    for (int i = 0; i < paths.size(); ++i)      
    {
      VisiLibity::Polygon hole;
      path_to_hole(paths[i], hole);
      visilibity_environment_.add_hole(hole);
    }
  }

  void Environment::create_visibility_graph(double epsilon)
  {
    if (paths_with_safety_margin_.size() > 0)
    {
      convert_to_visilibity_environment(paths_with_safety_margin_);
    }
    else
    {
      convert_to_visilibity_environment(paths_);
    }
    VisiLibity::Visibility_Graph visibility_graph(visilibity_environment_, epsilon);
  }
  
  VisiLibity::Polyline Environment::shortest_path(VisiLibity::Point start, VisiLibity::Point finish, double epsilon)
  {
    create_visibility_graph(epsilon);
    return visilibity_environment_.shortest_path(start, finish, epsilon);
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

    return is_valid_environment(ppg);

    return true;
  }

  bool Environment::is_valid_environment(ClipperLib::Paths &paths)
  {
    std::cout << "hei" << std::endl;
    std::cout << paths.size() << std::endl;

    for (int i = 0; i < paths.size()-1; ++i)
    {
      for (int j = i+1; j < paths.size(); ++j)
      {
        std::cout << i << ", " << j << std::endl;
        if(path_intersect(paths[i], paths[j]))
        {
          std::cout << "intersection!!!!!!!" << std::endl;

          return false;
        }
      }
    }
    return true;
  }

  bool Environment::path_intersect(const ClipperLib::Path &path1, const ClipperLib::Path &path2)
  {
    //Check if bounding boxes intersects
    ClipperLib::cInt x_max1, x_min1, y_max1, y_min1;
    ClipperLib::cInt x_min2, x_max2, y_max2, y_min2;
    find_max_and_min_in_path(path1, 'X', x_max1, x_min1);
    find_max_and_min_in_path(path1, 'Y', y_max1, y_min1);
    find_max_and_min_in_path(path2, 'X', x_max2, x_min2);
    find_max_and_min_in_path(path2, 'Y', x_max2, x_min2);

    ClipperLib::cInt width1 = abs(x_max1-x_min1);
    ClipperLib::cInt height1 = abs(y_max1-y_min1);
    ClipperLib::cInt width2 = abs(x_max2-x_min2);
    ClipperLib::cInt height2 = abs(y_max2-y_min2);


    return (abs(x_min1 - x_min2) * 2 < (width1 + width2)) &&
           (abs(y_min1 - y_min2) * 2 < (height1 + height2));

/*
    return !(x_min2 > x_max1
          || x_max2 < x_min1
          || y_max2 > y_min1
          || y_min2 < y_max1);*/

    //PointInPolygon(const IntPoint pt, const Path &poly);
  }

  void Environment::find_max_and_min_in_path(const ClipperLib::Path &path, char coordinate, ClipperLib::cInt &max_val, ClipperLib::cInt &min_val) //, ClipperLib::cInt &y_max, ClipperLib::cInt &y_min)
  {
    if (!(coordinate == 'X' || coordinate == 'Y'))
    {
      return;
    }

    max_val = (ClipperLib::cInt)std::numeric_limits<int>::min();
    min_val = (ClipperLib::cInt)std::numeric_limits<int>::max();

    ClipperLib::cInt current;
    for (int i = 0; i < path.size(); ++i)
    {
      if (coordinate == 'X')
        current = path[i].X;
      else
        current = path[i].Y;

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
    SVGBuilder svg;    
    svg.style.brushClr = 0x129C0000;
    svg.style.penClr = 0xCCFFA07A;
    svg.style.pft = ClipperLib::pftEvenOdd;

    if (paths_with_safety_margin_.size() > 0)
    {
      svg.AddPaths(paths_with_safety_margin_);
    }
    else
    {
      svg.AddPaths(paths_);
    }

    svg.SaveToFile(filename, 1,10);
  }
}
