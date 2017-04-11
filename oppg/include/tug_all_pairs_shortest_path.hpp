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
		All_pairs_shortest_path( Environment &environment);
    void write_to_file( std::vector<std::vector<int>> &apsp);

    std::vector<std::vector<int>> get_apsp_matrix() const {return apsp_;};

	private:
		std::vector<std::vector<Point>> optimal_paths;
		double epsilon_ = 0.001;
   //std::vector<Point> find_optimal_path_from_all_points( Point &goal,  Environment &environment);
    std::vector<std::vector<int>> find_optimal_path_from_all_points( Environment &environment);
    std::vector<std::vector<int>> apsp_;
	};
}
#endif