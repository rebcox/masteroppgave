#include "tug_all_pairs_shortest_path.hpp"
#include "fstream"
#include "limits"

namespace Tug
{
  All_pairs_shortest_path::All_pairs_shortest_path( Environment &env)
  {
    apsp_ = find_optimal_path_from_all_points(env);
    find_optimal_path_from_all_points_2(env, apsp2_);
    //write_to_file(apsp_);
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

  std::vector<std::vector<int>> 
  All_pairs_shortest_path::find_optimal_path_from_all_points( Environment &env)
  {
    std::vector<std::vector<int>> optimal_vertex(env.n());
    for (int i = 0; i < env.n(); ++i)
    {
      optimal_vertex[i] = std::vector<int>(env.n());
    }
    Polyline shortest_path_temp;

    int a = 0;
      for (std::map<int,Point>::iterator i = env.begin(); 
                                         i != env.end(); 
                                         ++i)
      {
        int b = 0;

        for (std::map<int,Point>::iterator j = env.begin(); 
                                           j != env.end(); 
                                           ++j)
        {
          if (a==b)
          {
            optimal_vertex[a][b] = 0;
          }
          else
          {
            A_star_search(i->second, //start point
                          j->second, //end point
                          env.points(),
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

  void All_pairs_shortest_path::find_optimal_path_from_all_points_2(
                                Environment &env, 
                                std::map<std::pair<int,int>, int> &apsp)
  {
    // map<pair<from, to>, go_via>>
    Polyline shortest_path_temp;
    apsp.clear();

      for (std::map<int,Point>::iterator i = env.begin(); i != env.end(); ++i)
      {
        for (std::map<int,Point>::iterator j = env.begin(); j != env.end(); ++j)
        {
          std::pair<int,int> pair_of_points(i->first, j->first);
          if (i->first == j->first)
          {
            apsp.insert(std::pair<std::pair<int,int>,int>(pair_of_points, 0));
            apsp_costs_.insert(std::pair<std::pair<int,int>,double>(pair_of_points, 0.0));
          }
          else
          {
            A_star_search(i->second, //start point
                          j->second, //end point
                          env.points(),
                          shortest_path_temp,
                          epsilon_);
            if (shortest_path_temp.size() > 0)
            {
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
