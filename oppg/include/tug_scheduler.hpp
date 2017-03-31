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
    Scheduler(std::vector<Tug::Polyline> &paths, const Environment &environment);
    void print_paths(std::vector<Polyline> &paths);
    std::vector<Polyline> get_paths(){return paths_;};
    void print_schedule(std::vector<std::vector<int>> &time_schedule);


  private:
    void sort_on_increasing_shortest_path_length(std::vector<Polyline> &paths);
    void schedule(std::vector<std::vector<int>> &time_schedule, Polyline &path, const Boat &tug);
    void make_time_schedule(std::vector<std::vector<int>> &time_schedule, std::vector<Polyline> &paths, const Environment &environment);

    bool is_available(std::vector<int> &time_schedule_point, int t);
    void set_tug(std::vector<int> &time_schedule_point, int id, int t);

    //Point position_at_time(time_t time, const Point &pt_now, const Polyline &path, float speed);
    double eucledian_distance(const Point &point1, const Point &point2);

    std::vector<Tug::Polyline> paths_;
  };
}