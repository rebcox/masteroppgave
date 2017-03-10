#ifndef TUG_ALL_PAIRS_SHORTEST_PATH_H
#define TUG_ALL_PAIRS_SHORTEST_PATH_H

#include "tug_environment.hpp"
#include "tug_a_star_search.hpp"

namespace Tug
{
	class All_pairs_shortest_path
	{
	public:
		All_pairs_shortest_path(const Environment &environment);
	private:
		std::vector<std::vector<Point>> shortest_path;
	};
}
#endif