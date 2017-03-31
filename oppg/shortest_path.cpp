#include "include/shortest_path.h"

namespace Tug
{
  Shortest_path::Shortest_path(Tug::Environment &environment, const Point &start, 
                              const Point &finish, Polyline &shortest_path)
  {

    Point second_to_last_point;
    Point second_point;

    if(!is_valid_start_and_end_points(start, finish, environment))
    {
      //std::cout << "Start and/or finish point within obstacle" << std::endl;
      return;
    }
    
    int index_start = point_within_safety_margin(start,environment);

    if (index_start > 0)
    {
      std::cout << "Start point within safety margin of obstacle " << index_start<< std::endl;
      second_point = calculate_point_on_boundary(start, 
        environment.visilibity_environment_with_safety_margin_[index_start], environment); //todo: maybe one off
    }

    int index_finish = point_within_safety_margin(finish,environment);
    if (index_finish > 0)
    {
      std::cout << "Finish point within safety margin of obstacle " << index_finish << std::endl;
      second_to_last_point = calculate_point_on_boundary(finish, 
        environment.visilibity_environment_with_safety_margin_[index_finish], environment); //todo: maybe one off
      std::cout << second_to_last_point << std::endl;
    }

    double epsilon = 0.001;

    const Point *start_point_to_a_star;
    const Point *finish_point_to_a_star;

    if (index_start > 0)
    {
      start_point_to_a_star = &second_point;
    }
    else
    {
      start_point_to_a_star = &start;
    }
    if (index_finish > 0)
    {
      finish_point_to_a_star = &second_to_last_point;
    }
    else
    {
      finish_point_to_a_star = &finish;
    }
    
    A_star_search(*start_point_to_a_star, 
                  *finish_point_to_a_star,
                  environment.points(), 
                  shortest_path, 
                  epsilon);

    if (shortest_path.size() > 0 and shortest_path[shortest_path.size()-1] != finish)
    {
      shortest_path.push_back(finish);
    }

    if (shortest_path.size() > 0 and shortest_path[0] != start)
    {
      shortest_path.reverse();
      shortest_path.push_back(start);
      shortest_path.reverse();
    }
    set_waypoints(shortest_path);
  }

  /*std::vector<bool> Shortest_path::points_touching_outer_boundary(Tug::Environment &environment) 
  {
    std::vector<bool> out;
    for (int i = 0; i < environment.n(); ++i)
    {
      if (point_is_on_outer_boundary(environment(i), environment))
      {
        out.push_back(true);
        //environment.mark_point_as_on_boundary(i);
      }
      else
      {
        out.push_back(false);
      }
    }
    return out;
  }*/

  Shortest_path::Shortest_path(const std::string &all_pairs_shortest_path)
  {
    std::cout << "Reading file: " << all_pairs_shortest_path << std::endl;

    if (!read_file(all_pairs_shortest_path))
    {
      std::cout << "Could not read file: " << all_pairs_shortest_path << std::endl;
    }

  }

  void Shortest_path::calculate_shortest_path(int start_id, int finish_id, Polyline &shortest_path, Environment &environment)
  {
    bool valid_path = true;
    std::vector<int> path;

    path.push_back(start_id);
    shortest_path.push_back(environment(start_id+3));

    int next_vertex = apsp_[finish_id-1][start_id-1];
    int last_vertex = 0;
    //TODO: -1
    while (next_vertex != last_vertex)
    {
      path.push_back(next_vertex);
      shortest_path.push_back(environment(next_vertex+3));
      last_vertex = next_vertex;
      next_vertex = apsp_[finish_id-1][last_vertex-1];
      if (next_vertex == -1)
      {
        path.clear();
        shortest_path.clear();
        valid_path = false;
        break;
      }
    } 
    if (valid_path)
    {
      shortest_path.push_back(environment(finish_id+3));
      path.push_back(finish_id);
    }


    //return path;
  }

  bool Shortest_path::read_file(const std::string &filename)
  {
    std::ifstream ifs(filename);
    if (!ifs) return false;
    std::string line;
    //std::vector<std::vector<int>> apsp;
    apsp_.clear();

    while (std::getline(ifs, line))
    {
      apsp_.push_back(std::vector<int>());
      std::stringstream ss(line);
      while (1)
      {
        char c = ss.peek();  
        if (c == ' ') 
        {
          ss.read(&c, 2); 
        }
        int id;
        if(!(ss >> id))
        {
          break;
        }
        apsp_.back().push_back(id);
      }
    }

    ifs.close();

    for (int i = 0; i < apsp_.size(); ++i)
    {
      for (int j = 0; j < apsp_[i].size(); ++j)
      {
        std::cout << apsp_[i][j] << "  ";
      }
      std::cout << std::endl;
    }
    return true;
  }


  bool Shortest_path::point_is_on_outer_boundary(const Point &point, const Tug::Environment &env)
  {
    //outer 
    if (point.x() == env.x_max-1 || point.x() == env.x_min+1 || point.x() == env.y_min+1 || point.y() == env.y_max-1)
    {
      return true;
    }
    return false;
  }
  
  bool Shortest_path::point_is_on_outer_boundary(const VisiLibity::Point &point, const Tug::Environment &env)
  {
    if (point.x() == env.x_max-1 || point.x() == env.x_min+1 || point.x() == env.y_min+1 || point.y() == env.y_max-1)
    {
      return true;
    }
    return false;
  }


  bool Shortest_path::is_valid_start_and_end_points(const Point &start, const Point &finish,
                                                    const Tug::Environment &environment)
  {
    ClipperLib::IntPoint start_clipperlib(round(start.x()), round(start.y())); //todo: round good enough?
    ClipperLib::IntPoint finish_clipperlib(round(finish.x()), round(finish.y()));

    for (int i = 1; i < environment.paths_.size(); ++i) //path_[0] is outer boundary
    {
      if (ClipperLib::PointInPolygon(start_clipperlib, environment.paths_[i])>0)
      {
        return false;
      }
      if (ClipperLib::PointInPolygon(finish_clipperlib, environment.paths_[i])>0)
      {
        return false;
      }
    }
    return true;
  }

  int Shortest_path::point_within_safety_margin(const Point &point, const Tug::Environment &environment)
  {
    if (environment.paths_with_safety_margin_.size() == 0)
    {
      return 0;
    }
    ClipperLib::IntPoint point_clipperlib(round(point.x()), round(point.y()));
    for (int i = 1; i < environment.paths_with_safety_margin_.size(); ++i) //path_[0] is outer boundary
    {
      if (ClipperLib::PointInPolygon(point_clipperlib, environment.paths_with_safety_margin_[i])>0)
      {
        return i;
      }
    }
    return 0;
  }

  Point Shortest_path::calculate_point_on_boundary(const Point &point, const VisiLibity::Polygon &hole, const Tug::Environment &env)
  {
    Point current_shortest;
    Point current;
    double shortest_distance = std::numeric_limits<double>::max();

    if (!( point_is_on_outer_boundary(hole[hole.n()-1], env) and point_is_on_outer_boundary(hole[0], env) ))
    {
      VisiLibity::Line_Segment line(hole[hole.n()-1], hole[0], 0);
      current_shortest = point_closest_to_line_segment(point, line);
      shortest_distance = VisiLibity::distance(point, current_shortest);
    }

    for (int i = 0; i < hole.n()-1; ++i)
    {
      if (!( point_is_on_outer_boundary(hole[i], env) and point_is_on_outer_boundary(hole[i+1], env) ))
      {
        VisiLibity::Line_Segment line(hole[i], hole[i+1], 0);
        current = point_closest_to_line_segment(point, line);
        double dist = distance(point, current);
        if (dist < shortest_distance)
        {
          //std::cout << "distance: " << dist << std::endl;
          current_shortest = current;
          shortest_distance = dist;
        }
      }

    }
    current_shortest.create_visibility_polygon(env.visilibity_environment());
    return current_shortest; //point_closest_to_line_segment(point, line_segments[shortest]);
  }

  Point Shortest_path::point_closest_to_line_segment(const Point &point, const VisiLibity::Line_Segment &line)
  {
    VisiLibity::Point pt = point.projection_onto(line);
    return Point(pt);
  }

  void Shortest_path::set_waypoints(Polyline &shortest_path)
  {
    for (int i = 0; i < shortest_path.size(); ++i)
    {
      waypoints_.push_back(shortest_path[i]);
    }
  }
  
  std::vector<Waypoint> Shortest_path::get_waypoints()
  {
    return waypoints_;
  }

}

