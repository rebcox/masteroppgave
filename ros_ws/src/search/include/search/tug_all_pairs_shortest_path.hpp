#ifndef TUG_ALL_PAIRS_SHORTEST_PATH_H
#define TUG_ALL_PAIRS_SHORTEST_PATH_H

#include "tug_a_star_search.hpp"
#include "vector"
#include "map"

namespace Tug
{
	class All_pairs_shortest_path
	{
	public:
		All_pairs_shortest_path( Environment &environment);
    void write_to_file( std::vector<std::vector<int>> &apsp);

    std::vector<std::vector<int>> get_apsp_matrix() const {return apsp_;};
    std::map<std::pair<int,int>, int> get_apsp_matrix_2() const {return apsp2_;};
    std::map<std::pair<int,int>, double> get_apsp_costs() const {return apsp_costs_;};

	private:
		std::vector<std::vector<Point>> optimal_paths;
		double epsilon_ = 0.001;
    void find_optimal_path_from_all_points_2(Environment &environment, std::map<std::pair<int,int>, int> &apsp);
    std::vector<std::vector<int>> find_optimal_path_from_all_points(Environment &environment);
    std::vector<std::vector<int>> apsp_;
    std::map<std::pair<int,int>, int> apsp2_;
    std::map<std::pair<int,int>, double> apsp_costs_;
	};
}
#endif
