#include "../include/tug_environment.hpp"
#include <sstream>
#include <limits>
#include "../../search/include/tug_shortest_path.hpp"


namespace Tug
{
  Environment::Environment(const std::string& filename, double scale, double epsilon)
  {
    epsilon_ = epsilon;
    ClipperLib::Paths temp_paths;
    bool ok = load_from_file(temp_paths, filename, scale);
    clip_against_outer_boundary(temp_paths, paths_);

    if(!ok)
    {
      std::cout << "Could not read environment file" << std::endl;
    }
    convert_to_visilibity_environment(paths_, visilibity_environment_);

    if (!visilibity_environment_.is_valid(epsilon))
    {
      std::cout << "Environment is not valid" << std::endl;
    }
    
    update_tug_point_list(paths_, points_in_environment_);
  }

  std::map<int, Point>::iterator Environment::begin()
  {
    if (environment_has_safety_margin)
    {
      return points_in_environment_with_safety_margin_.begin();
    }
    else
    {
      return points_in_environment_.begin();
    }
  }

  std::map<int, Point>::iterator Environment::end()
  {
    if (environment_has_safety_margin)
    {
      return points_in_environment_with_safety_margin_.end();
    }
    else
    {
      return points_in_environment_.end();
    }
  }

  std::map<int, Point>::const_iterator Environment::const_begin() const
  {
    if (environment_has_safety_margin)
    {
      return points_in_environment_with_safety_margin_.begin();
    }
    else
    {
      return points_in_environment_.begin();
    }
  }

  std::map<int, Point>::const_iterator Environment::const_end() const
  {
    if (environment_has_safety_margin)
    {
      return points_in_environment_with_safety_margin_.end();
    }
    else
    {
      return points_in_environment_.end();
    }
  }

  std::vector<Point> Environment::points()
  {
    std::vector<Point> dummy_points;
    for (std::map<int,Point>::iterator pt = begin(); pt != end(); ++pt)
    {
      dummy_points.push_back(pt->second);
    }
    return dummy_points;
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
      return points_in_environment_with_safety_margin_.size();
    }
    else
    {
      return points_in_environment_.size();
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

    environment_has_safety_margin = true;
    update_tug_point_list(paths_with_safety_margin_, points_in_environment_with_safety_margin_);
  }
  
  bool Environment::point_is_within_outer_boundary(const Tug::Point point)
  {
    if (point.x() >= x_max_-1 || point.x() <= x_min_+1 || point.y() <= y_min_+1 || point.y() >= y_max_-1) 
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

  void Environment::update_tug_point_list(const ClipperLib::Paths &paths, std::map<int, Point> &tug_points)
  {
    tug_points.clear();

    for (int i = 1; i < paths.size(); ++i)
    {
      for (int j = 0; j < paths[i].size(); ++j)
      {
        int id = ++id_counter_;
        tug_points.insert(std::pair<int,Point>(id, Point(paths[i][j], *this, id)));
        coordinate_to_id.insert(std::pair<std::pair<double,double>,int>
                              (std::make_pair(paths[i][j].X, paths[i][j].Y), id));
      }
    }
    mark_points_touching_outer_boundary();
    make_visibility_graphs_for_points(tug_points);
  }
  void Environment::print_coordinates_and_id() const
  {
    for (auto i = coordinate_to_id.begin(); i != coordinate_to_id.end(); ++i)
    {
      std::cout << i->second << ": " << i->first.first << ", " << i->first.second << std::endl;
    }
  }

  void Environment::make_visibility_graphs_for_points(std::map<int, Point> &tug_points)
  {
    for (std::map<int, Point>::iterator i = tug_points.begin(); i != tug_points.end(); ++i)
    {
      i->second.create_visibility_polygon(*this);
    }
  }

  Polyline Environment::shortest_path(const Point &start, const Point &finish) //, double epsilon)
  {
    Polyline shortest_path;

    Shortest_path((*this), start, finish, shortest_path);
    return shortest_path;
  }

  void Environment::set_outer_boundary(const ClipperLib::Path &outer_boundary, VisiLibity::Environment &environment)
  {
    find_max_and_min_in_path(outer_boundary, 'X', x_max_, x_min_);
    find_max_and_min_in_path(outer_boundary, 'Y', y_max_, y_min_);

    VisiLibity::Polygon outer_boundary_polygon;
    path_to_hole(outer_boundary, outer_boundary_polygon);
    environment.set_outer_boundary(outer_boundary_polygon);
  }

  void Environment::mark_points_touching_outer_boundary() 
  {

    for (std::map<int,Point>::iterator pt = begin(); pt != end(); ++pt)
    {
      if (environment_has_safety_margin and point_is_on_outer_boundary(pt->second))
      {
        pt->second.is_on_outer_boundary = true;
      }
      else if(!environment_has_safety_margin and point_is_on_outer_boundary(pt->second))
      {
        pt->second.is_on_outer_boundary = true;        
      }
    } 
  }

  bool Environment::point_is_on_outer_boundary(const VisiLibity::Point &point) const
  {
    //outer boundary is 1 unit smaller
    if (point.x() == x_max_-1 || point.x() == x_min_+1 || point.y() == y_min_+1 || point.y() == y_max_-1)
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

    return true;
  }

  void Environment::find_max_and_min_in_path(const ClipperLib::Path &path, char coordinate, int &max_val, int &min_val) 
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

  int Environment::find_id(const VisiLibity::Point &point) const
  {
    try
    {
      return coordinate_to_id.at(std::make_pair(point.x(),point.y()));
    }
    catch(...)
    {
      return -1;
    }

  }

  void Environment::get_boundaries(int &x_min_out, int &x_max_out, int &y_min_out, int &y_max_out) const
  {
    x_min_out = x_min_+1;
    x_max_out = x_max_-1;
    y_min_out = y_min_+1;
    y_max_out = y_max_-1;
  }

  void Environment::save_environment_as_svg(const std::string filename)
  {
    Polyline dummy;
    save_environment_as_svg(filename, dummy);
  }

  void Environment::save_environment_as_svg(const std::string filename, const Polyline &shortest_path)
  {
    std::vector<Polyline> shortest_paths;
    shortest_paths.push_back(shortest_path);
    save_environment_as_svg(filename, shortest_paths);
  }

  void Environment::save_environment_as_svg(const std::string filename, const std::vector<Polyline> &shortest_paths)
  {
    SVGBuilder svg;    
    svg.style.brushClr = 0x129C0000;
    svg.style.penClr = 0xCCFFA07A;
    svg.style.pft = ClipperLib::pftEvenOdd;

    if (environment_has_safety_margin)
    {
      svg.AddPaths(paths_with_safety_margin_);
    }
    svg.AddPaths(paths_);

    for (int i = 0; i < shortest_paths.size(); ++i)
    {
      if (shortest_paths[i].size()>0)
      {
        svg.AddPolyline(shortest_paths[i]);
      }
    }
    svg.SaveToFile(filename, 1, 0);
  }
}
