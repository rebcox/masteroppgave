#include <vector>
#include "time.h"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"

namespace Tug
{
  class Scheduler
  {
  public:
    Scheduler(const std::vector<Tug::Polyline> &paths, const Environment &environment);
    void print_paths();
  private:
    void sort_on_increasing_shortest_path_length(std::vector<Polyline> &paths);
    void schedule(std::vector<std::vector<int>> &time_schedule, const Polyline &path);
    void make_time_schedule(const std::vector<Polyline> &paths, const Environment &environment);

    //Point position_at_time(time_t time, const Point &pt_now, const Polyline &path, float speed);
    //double eucledian_distance(const Point &point1, const Point &point2);

    std::vector<Tug::Polyline> paths_;
  };
}