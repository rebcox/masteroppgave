#ifndef TUG_ALL_PAIRS_SHORTEST_PATH_H
#define TUG_ALL_PAIRS_SHORTEST_PATH_H

#include "tug_environment.hpp"
#include "tug_a_star_search.hpp"
#include "vector"

namespace Tug
{
	class All_pairs_shortest_path
	{
	public:
		All_pairs_shortest_path(const Environment &environment);
    void write_to_file(const std::vector<std::vector<int>> &apsp);
	private:
    int get_point_number(const VisiLibity::Point &point, const Environment &environment);
		std::vector<std::vector<Point>> optimal_paths;
		double epsilon_ = 0.001;
   //std::vector<Point> find_optimal_path_from_all_points(const Point &goal, const Environment &environment);
  std::vector<std::vector<int>> find_optimal_path_from_all_points(const Environment &environment);

	};
}
#endif