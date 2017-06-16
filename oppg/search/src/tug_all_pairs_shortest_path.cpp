#include "../include/tug_all_pairs_shortest_path.hpp"
#include "fstream"
#include "limits"

namespace Tug
{
  All_pairs_shortest_path::All_pairs_shortest_path( Environment &environment)
  {
    apsp_ = find_optimal_path_from_all_points(environment);
    find_optimal_path_from_all_points_2(environment, apsp2_);
    //write_to_file(apsp_);
    write_to_file(apsp2_);

  }

  void All_pairs_shortest_path::write_to_file( std::vector<std::vector<int>> &apsp)
  {
    std::ofstream file_output;
    file_output.open ("all_pairs_shortest_path.txt");
    for (int i = 0; i < apsp.size(); i++)
    {
      for (int j = 0; j < apsp[i].size(); ++j)
      {
        file_output << apsp[i][j] << "  ";
      }
      file_output << "\n";
    }
    std::cout << "All pairs shortest path written to file\n";
    file_output.close();
  }

  void All_pairs_shortest_path::write_to_file(std::map<std::pair<int,int>, int> &apsp)
  {
    std::ofstream file_output;
    file_output.open ("all_pairs_shortest_path2.txt");

    for (std::map<std::pair<int,int>, int>::iterator i = apsp.begin(); i != apsp.end(); ++i)
    {
      file_output << "(" << i->first.first << " to " << i->first.second << ") go via " << i->second << "\n";
    }
    std::cout << "All pairs shortest path written to file\n";
    file_output.close();
  }

  std::vector<std::vector<int>> All_pairs_shortest_path::find_optimal_path_from_all_points( Environment &environment)
  {
    std::vector<std::vector<int>> optimal_vertex(environment.n());
    for (int i = 0; i < environment.n(); ++i)
    {
      optimal_vertex[i] = std::vector<int>(environment.n());
    }
    Polyline shortest_path_temp;

    int a = 0;
      for (std::map<int,Point>::iterator i = environment.begin(); i != environment.end(); ++i)
      {
        int b = 0;

        for (std::map<int,Point>::iterator j = environment.begin(); j != environment.end(); ++j)
        {
          if (a==b)
          {
            optimal_vertex[a][b] = 0;
          }
          else
          {
            A_star_search(i->second, //start point
                          j->second, //end point
                          environment.points(),
                          shortest_path_temp,
                          epsilon_);
            if (shortest_path_temp.size() > 0)
            {
              optimal_vertex[a][b] = shortest_path_temp[1].id(); 
            }
            else
            {
              optimal_vertex[a][b] = -1;
            }
          }
          b++;
        }
        a++;
      }
      return optimal_vertex;

  }

  void All_pairs_shortest_path::find_optimal_path_from_all_points_2( Environment &environment, std::map<std::pair<int,int>, int> &apsp)
  {
    // map<pair<from, to>, go_via>>
    Polyline shortest_path_temp;
    apsp.clear();
    std::vector<Point> points = environment.points();

      for (std::map<int,Point>::iterator i = environment.begin(); i != environment.end(); ++i)
      {
       // std::cout << i->first << ": " << i->second << std::endl;

        for (std::map<int,Point>::iterator j = environment.begin(); j != environment.end(); ++j)
        {
          std::pair<int,int> pair_of_points(i->first, j->first);
          if (i->first == j->first)
          {
            apsp.insert(std::pair<std::pair<int,int>,int>(pair_of_points, 0));
            apsp_costs_.insert(std::pair<std::pair<int,int>,double>(pair_of_points, 0.0));
          }
          else
          {
            shortest_path_temp.clear();

            A_star_search(i->second, //start point
                          j->second, //end point
                          points,
                          shortest_path_temp,
                          epsilon_);
            if (shortest_path_temp.size() > 0)
            {
              /*for (int i = 0; i < shortest_path_temp.size(); ++i)
              {
                std::cout << shortest_path_temp[i] << ", ";
              }*/
              //std::cout << "inserted: " << shortest_path_temp[1].id() << " which is " << shortest_path_temp[1] << std::endl;
              apsp.insert(std::pair<std::pair<int,int>,int>(pair_of_points, shortest_path_temp[1].id()));
              apsp_costs_.insert(std::pair<std::pair<int,int>,double>(pair_of_points, shortest_path_temp.length()));
            }
            else
            {
              apsp.insert(std::pair<std::pair<int,int>,int>(pair_of_points, -1));
              apsp_costs_.insert(std::pair<std::pair<int,int>,double>(pair_of_points, std::numeric_limits<double>::max()));
            }
          }
        }
      }
  }
}
