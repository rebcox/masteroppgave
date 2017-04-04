#include <vector>
#include "time.h"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"
#include "tug_boat.hpp"

namespace Tug
{
  class Scheduler
  {
  public:
    Scheduler(std::vector<Boat> &tugs, const Environment &environment);
    void print_paths(std::vector<Boat> &tugs);
    //std::vector<Polyline> get_paths(){return paths_;};
    void print_schedule();
    std::vector<std::vector<int>> get_time_schedule(){return time_schedule_;};

  private:
    std::vector<std::vector<int>> time_schedule_;
    void sort_on_increasing_shortest_path_length(std::vector<Polyline> &paths);
    void sort_on_increasing_shortest_path_length( std::vector<Boat> &tugs);

    void schedule(std::vector<std::vector<int>> &time_schedule, const Boat &tug);
    void make_time_schedule(std::vector<std::vector<int>> &time_schedule,
                            std::vector<Boat> &tugs, const Environment &environment);

    bool is_available(std::vector<int> &time_schedule_point, int t);
    void set_tug(std::vector<int> &time_schedule_point, int id, int t);

    //Point position_at_time(time_t time, const Point &pt_now, const Polyline &path, float speed);
    double eucledian_distance(const Point &point1, const Point &point2);

    //std::vector<Tug::Polyline> paths_;
  };
}