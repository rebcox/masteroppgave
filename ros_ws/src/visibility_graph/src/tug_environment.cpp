#include "tug_environment.h"
#include <sstream>


namespace Tug
{
  Environment::Environment(const std::string& filename, double scale)
  {
    bool ok = load_from_file(paths_, filename, scale);
    std::cout << "Number of objects: " << paths_.size() << std::endl;
    if(!ok)
    {
      std::cout << "Could not read environment file" << std::endl;
    }
    /*std::ifstream fin(filename.c_str());
    //if(fin.fail()) { std::cerr << "\x1b[5;31m" << "Input file
    //opening failed." << "\x1b[0m\n" << "\a \n"; exit(1);}
    assert( !fin.fail() );

    //Temporary vars for numbers to be read from file.
    double x_temp, y_temp;  
    std::vector<VisiLibity::Point> vertices_temp;

    //Skip comments
    while( fin.peek() == '/' ) 
      fin.ignore(200,'\n');

    //Read outer_boundary.
    while ( fin.peek() != '/' ){
      fin >> x_temp >> y_temp;
      //Skip to next line.
      fin.ignore(1);
      if( fin.eof() )
  { 
    outer_boundary_.set_vertices(vertices_temp);
    fin.close(); 
    return;
  }      
      vertices_temp.push_back( VisiLibity::Point(x_temp, y_temp) );
    }
    outer_boundary_.set_vertices(vertices_temp);
    vertices_temp.clear();
    
    //Read holes.
    VisiLibity::Polygon polygon_temp;
    while(1){
      //Skip comments
      while( fin.peek() == '/' )
  fin.ignore(200,'\n');
      if( fin.eof() )
  { fin.close(); return; }
      while( fin.peek() != '/' ){ 
  fin >> x_temp >> y_temp;
  if( fin.eof() )
    { 
      polygon_temp.set_vertices(vertices_temp);
      holes_.push_back(polygon_temp);
      fin.close(); 
      return;
    }
  vertices_temp.push_back( VisiLibity::Point(x_temp, y_temp) );
  //Skips to next line.
  fin.ignore(1);
      }
      polygon_temp.set_vertices(vertices_temp);
      holes_.push_back(polygon_temp);
      vertices_temp.clear();
    }*/

  }

  void Environment::add_constant_safety_margin(double margin) //, ClipperLib::Paths &solution)
  {
    ClipperLib::ClipperOffset co;
    for (int i = 0; i < paths_.size(); ++i)
    {
      co.AddPath(paths_[i], ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    }
    co.Execute(paths_with_safety_margin_, margin);

/*    for (int i = 0; i < holes_.size(); ++i)
    {
      offset_polygon(holes_[i], margin);
    }
    //for each hole, add margin around by calling dilate
  */
  }

  void Environment::offset_polygon(VisiLibity::Polygon &polygon, int margin)
  {
    //Polygon is defined clock wise.
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
    return true;
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
