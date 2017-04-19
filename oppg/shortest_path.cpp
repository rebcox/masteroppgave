#include "include/shortest_path.h"
#include "limits"

namespace Tug
{
  /*
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
*/

  Shortest_path::Shortest_path(Tug::Environment &environment, const Point &start, 
                              const Point &finish, Polyline &shortest_path)
  {
    All_pairs_shortest_path all_pairs_shortest_path(environment);
    apsp_ = all_pairs_shortest_path.get_apsp_matrix();
    apsp2_ = all_pairs_shortest_path.get_apsp_matrix_2();

    Point second_to_last_point;
    Point second_point;

    if(!is_valid_start_and_end_points(start, finish, environment))
    {
      //std::cout << "Start and/or finish point within obstacle" << std::endl;
      return;
    }
    
    const Point *start_point_to_a_star;
    const Point *finish_point_to_a_star;

    int index_start = point_within_safety_margin(start,environment);

    if (index_start > 0)
    {
      std::cout << "Start point within safety margin of obstacle " << index_start<< std::endl;
      second_point = calculate_point_on_boundary(start, 
        environment.visilibity_environment_with_safety_margin_[index_start], environment); //todo: maybe one off
      std::cout << "new point " << second_point << std::endl;
     // second_point.create_visibility_polygon(environment);
      std::cout << "size vector " << second_point.visible_vertices().size() << std::endl;
      std::cout << "size polygon " << second_point.visibility_polygon().n() << std::endl;
      VisiLibity::Polygon vp = second_point.visibility_polygon();
      for (int i = 0; i < vp.n(); ++i)
      {
        std::cout << "vispoly: " << vp[i] << std::endl;
      }
      std::vector<int> sec_p = second_point.visible_vertices();

      for (int i = 0; i < sec_p.size(); ++i)
      {
        std::cout << "vis: " << environment(sec_p[i]) << std::endl;
      }
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
    
   /*A_star_search(*start_point_to_a_star, 
                  *finish_point_to_a_star,
                  environment.points(), 
                  shortest_path, 
                  epsilon);*/

    /*Point pt1(start_point_to_a_star->x(), start_point_to_a_star->y(), environment);
    Point pt2(finish_point_to_a_star->x(), finish_point_to_a_star->y(), environment);

    std::cout << "pt1 " << pt1.visible_vertices_.size() << std::endl;

    bool ok = calculate_shortest_path(pt1, 
                                      pt2,
                                      shortest_path,
                                      environment);*/

    bool ok = calculate_shortest_path(*start_point_to_a_star, 
                                      *finish_point_to_a_star,
                                      shortest_path,
                                      environment);

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

    /*for (int i = 0; i < shortest_path.size(); ++i)
    {
      std::cout << shortest_path[i] << std::endl;
    }*/

    //delete start_point_to_a_star;
    //delete finish_point_to_a_star;
  }

  Shortest_path::Shortest_path(Environment &environment)
  {
    All_pairs_shortest_path all_pairs_shortest_path(environment);
    apsp_ = all_pairs_shortest_path.get_apsp_matrix();
    apsp2_ = all_pairs_shortest_path.get_apsp_matrix_2();
  }


  Shortest_path::Shortest_path(const std::string &all_pairs_shortest_path, Environment &environment)
  {
    std::cout << "Reading file: " << all_pairs_shortest_path << std::endl;

    if (!read_file(all_pairs_shortest_path))
    {
      std::cout << "Could not read file: " << all_pairs_shortest_path << std::endl;
    }
  }

  double cost(const Point &pt1, const Point &pt2)
  {
    return sqrt(pow(pt1.x() - pt2.x(), 2) + pow(pt1.y() - pt2.y(), 2));
  }

  bool Shortest_path::calculate_shortest_path(const Point &start, const Point &end, Polyline &shortest_path, Environment &environment)
  {
    bool valid_path = true;

    //VisiLibity::Visibility_Polygon start_vp = start.visibility_polygon();
    shortest_path.clear();
    std::vector<int> visible_vertices_start = start.visible_vertices();
    std::vector<int> visible_vertices_end = end.visible_vertices();


    if (start.is_visible(end))
    {
      shortest_path.push_back(start);
      shortest_path.push_back(end);
      return true;
    }

    std::vector<std::pair<int, double>> costs_from_start;
    std::vector<std::pair<int, double>> costs_from_end;

    //calculate cost from start and finish points to all points in their respectively visibility polygon
    for (int i = 0; i < visible_vertices_start.size(); ++i)
    {
      int id = visible_vertices_start[i];
      costs_from_start.push_back(std::pair<int,double>(id, cost(start, environment(id))));
    }
    for (int i = 0; i < visible_vertices_end.size(); ++i)
    {
      int id = visible_vertices_end[i];
      costs_from_end.push_back(std::pair<int,double>(id, cost(end, environment(id))));      
    }

    double lowest_cost = std::numeric_limits<double>::max();
    int best_start = -1;
    int best_end = -1;

    for (int i = 0; i < costs_from_start.size(); ++i) //Loop through all points seen by start
    {
      int start_id = costs_from_start[i].first;

      //Find shortest path from current point seen from start to all points seen by finish
      for (int j = 0; j < costs_from_end.size(); ++j) 
      {
        int end_id = costs_from_end[j].first;
        
        Polyline shortest_path_internal;

        bool path_found = calculate_shortest_path(start_id, end_id, shortest_path_internal, environment);

        if (path_found)
        {
         // if ((costs_from_start[i].second + environment(start_id).get_cost_to(end_id) + costs_from_end[j].second) < lowest_cost)
          double current_cost = costs_from_start[i].second + shortest_path_internal.length() + costs_from_end[j].second;
          if (current_cost < lowest_cost)
          {
            best_start = start_id;
            best_end = end_id;
            lowest_cost = current_cost;
          }
        }
      }
    }

    if (best_start > -1 and best_end > -1)
    {
      shortest_path.push_back(start);
      Polyline shortest_path_internal;
      calculate_shortest_path(best_start, best_end, shortest_path_internal, environment);

      for (int i = 0; i < shortest_path_internal.size(); ++i)
      {
        shortest_path.push_back(shortest_path_internal[i]);
      }
      shortest_path.push_back(end);
      return true;
    }
    return false;


    /*
    double shortest_from_start;
    

    if(start_vp.n()>0)
    {
     shortest_from_start = cost(start, start_vp[0]);      
    }
    else
    {
      return false;
    }
    int best_id_start = 0;
    for (int i = 1; i < start_vp.n(); ++i)
    {
      if (cost(start, start_vp[i]) < shortest_from_start)
      {
        best_id_start = i;
      }
    }

    shortest_path.push_back(environment(best_id_start));  

    VisiLibity::Visibility_Polygon end_vp = end.visibility_polygon();
    double shortest_from_end;
    if(end_vp.n()>0)
    {
     shortest_from_end = cost(end, end_vp[0]);      
    }
    else
    {
      return false;
    }
    int best_id_end = 0;
    for (int i = 1; i < end_vp.n(); ++i)
    {
      if (cost(end, end_vp[i]) < shortest_from_end)
      {
        best_id_end = i;
      }
    }

    bool path_found = calculate_shortest_path(best_id_start, best_id_end, shortest_path, environment);
    shortest_path.push_back(end);

    return path_found;*/

  }

  bool Shortest_path::calculate_shortest_path(int start_id, int finish_id, Polyline &shortest_path, Environment &environment)
  {
    /*bool valid_path = true;
   // std::vector<int> path;
    shortest_path.clear();
   // path.push_back(start_id);
    shortest_path.push_back(environment(start_id));

    int next_vertex = apsp_[finish_id-1][start_id-1];

    int prev_vertex = 0;

    while (next_vertex != prev_vertex)
    {
      //path.push_back(next_vertex);
      shortest_path.push_back(environment(next_vertex));
      prev_vertex = next_vertex;

      //HER ER FEIL


      next_vertex = apsp_[finish_id-1][prev_vertex-1];
      if (next_vertex == -1)
      {
        //path.clear();
        shortest_path.clear();
        valid_path = false;
        return valid_path;
      }
    } 
    if (valid_path)
    {
      //shortest_path.push_back(environment(finish_id));
      //path.push_back(finish_id);
    }
    return valid_path;*/
    
    shortest_path.clear();
    shortest_path.push_back(environment(start_id));

    //First vertex to visit. If it is equal to start_id, than there are not more points to traverse
    int next_vertex = apsp2_[std::make_pair(finish_id, start_id)];

    if (next_vertex == -1)
    {
      shortest_path.clear();
      return false;      
    }
    int prev_vertex = 0;

    while (next_vertex != prev_vertex) 
    {
      shortest_path.push_back(environment(next_vertex));

      prev_vertex = next_vertex;
      next_vertex = apsp2_[std::make_pair(finish_id, prev_vertex)];

      if (next_vertex == -1)
      {
        shortest_path.clear();
        return false;
      }
    }
    return true;
  }

  bool Shortest_path::read_file(const std::string &filename)
  {
    std::ifstream ifs(filename);
    if (!ifs) return false;
    std::string line;
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

    if (!( env.point_is_on_outer_boundary(hole[hole.n()-1]) and env.point_is_on_outer_boundary(hole[0]) ))
    {
      VisiLibity::Line_Segment line(hole[hole.n()-1], hole[0], 0);
      current_shortest = point_closest_to_line_segment(point, line);
      shortest_distance = VisiLibity::distance(point, current_shortest);
    }

    for (int i = 0; i < hole.n()-1; ++i)
    {
      if (!( env.point_is_on_outer_boundary(hole[i]) and env.point_is_on_outer_boundary(hole[i+1]) ))
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
   std::cout << "fra her: "  << std::endl;
   Point pt(current_shortest.x(), current_shortest.y(), env);

   // current_shortest.create_visibility_polygon(env);
   std::cout << "til her "  << std::endl;

    return pt; //current_shortest; //point_closest_to_line_segment(point, line_segments[shortest]);
  }

  Point Shortest_path::point_closest_to_line_segment(const Point &point, const VisiLibity::Line_Segment &line)
  {
    VisiLibity::Point pt = point.projection_onto(line);
    return Point(pt);
  }
}

