#include "include/tug_all_pairs_shortest_path.hpp"
#include "fstream"
namespace Tug
{
  All_pairs_shortest_path::All_pairs_shortest_path( Environment &environment)
  {
    apsp_ = find_optimal_path_from_all_points(environment);
    write_to_file(apsp_);
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

  //std::vector<Point> All_pairs_shortest_path::find_optimal_path_from_all_points( Point &goal,  Environment &environment)
  std::vector<std::vector<int>> All_pairs_shortest_path::find_optimal_path_from_all_points( Environment &environment)
  {
    //std::vector<std::vector<double>> shortest_path_cost(environment.n()-4);
    std::vector<std::vector<int>> optimal_vertex(environment.n());
    for (int i = 0; i < environment.n(); ++i)
    {
      optimal_vertex[i] = std::vector<int>(environment.n());
      //shortest_path_cost[i] = std::vector<double>(environment.n()-4);
    }
    //int optimal_vertex[environment.n()][environment.n()];
    Polyline shortest_path_temp;

//      for (int i = 0; i < environment.n(); ++i)
    int a=0;
      for (std::map<int,Point>::const_iterator i = environment.const_begin(); i != environment.const_end(); ++i)
      {
        int b =0;

        for (std::map<int,Point>::const_iterator j = environment.const_begin(); j != environment.const_end(); ++j)
       // for (int j = 0; j < environment.n(); ++j)
        {
          if (a==b)
          {
            optimal_vertex[a][b] = 0;
          }
          else
          {
            A_star_search(i->second, //start
                          j->second, //finish
                          environment.points(),
                          shortest_path_temp,
                          epsilon_);
            if (shortest_path_temp.size() > 0)
            {
              //environment(i+4).add_shortest_path_cost(environment(j+4), shortest_path_temp.length());
              optimal_vertex[a][b] = shortest_path_temp[1].id(); //get_point_number(shortest_path_temp[1], environment); 
              //shortest_path_cost[i][j] = shortest_path_temp.length();
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

}