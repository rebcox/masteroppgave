#include "include/tug_all_pairs_shortest_path.hpp"
#include "fstream"
namespace Tug
{
  All_pairs_shortest_path::All_pairs_shortest_path(const Environment &environment)
  {
    std::vector<std::vector<int>> apsp = find_optimal_path_from_all_points(environment);
    write_to_file(apsp);
  }

  void All_pairs_shortest_path::write_to_file(const std::vector<std::vector<int>> &apsp)
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

    file_output.close();
  }



  int All_pairs_shortest_path::get_point_number(const VisiLibity::Point &point, const Environment &environment)
  {
    for (int i = 0; i < environment.n(); ++i)
    {
      if (point == environment(i))
      {
        return i+1-4;
      }
    }
    return -1;
  }

  //std::vector<Point> All_pairs_shortest_path::find_optimal_path_from_all_points(const Point &goal, const Environment &environment)
  std::vector<std::vector<int>> All_pairs_shortest_path::find_optimal_path_from_all_points(const Environment &environment)
  {

    std::vector<std::vector<int>> optimal_vertex(environment.n()-4);
    for (int i = 0; i < environment.n()-4; ++i)
    {
      optimal_vertex[i] = std::vector<int>(environment.n()-4);
    }
    //int optimal_vertex[environment.n()][environment.n()];
    Polyline shortest_path_temp;

      for (int i = 0; i < environment.n()-4; ++i)
      {
        for (int j = 0; j < environment.n()-4; ++j)
        {
          if (i==j)
          {
            optimal_vertex[i][j] = 0;
          }
          else
          {
            A_star_search(environment(i+4), //start
                          environment(j+4), //finish
                          environment.points(),
                          shortest_path_temp,
                          epsilon_);
            if (shortest_path_temp.size() > 0)
            {
              optimal_vertex[i][j] = shortest_path_temp[1].id(); //get_point_number(shortest_path_temp[1], environment); 
            }
            else
            {
              optimal_vertex[i][j] = -1;
            }
          }
        }
      }
      return optimal_vertex;

  }

}