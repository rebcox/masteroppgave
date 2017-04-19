#include <vector>
#include "time.h"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"
#include "tug_boat.hpp"
#include "tug_waypoint.hpp"

namespace Tug
{
  class Scheduler
  {
  public:
    Scheduler(std::vector<Boat> &tugs, const Environment &environment);
    void print_paths(std::vector<Boat> &tugs);
    void print_schedule();
    std::vector<std::vector<int>> get_time_schedule(){return time_schedule_;};

  private:
    std::vector<std::vector<int>> time_schedule_;
    void sort_on_increasing_shortest_path_length(std::vector<Polyline> &paths);
    void sort_on_increasing_shortest_path_length( std::vector<Boat> &tugs);

    void schedule(std::vector<std::vector<int>> &time_schedule, const Boat &tug, std::vector<Waypoint> &waypoints);
    void make_time_schedule(std::vector<std::vector<int>> &time_schedule,
                            std::vector<Boat> &tugs, const Environment &environment);
    bool time_schedule_is_zero(std::vector<int> &time_schedule, int t, int duration);

    bool is_available(std::vector<std::vector<int>> &time_schedule, 
                                std::vector<Waypoint> &waypoints,
                                int id,
                                int t,
                                int duration);
    void set_tug(std::vector<int> &time_schedule_point, int id, int t, int duration);

    double eucledian_distance(const Point &point1, const Point &point2);
  };
}