#include "include/tug_scheduler.hpp"

namespace Tug
{
  Scheduler::Scheduler(const std::vector<Polyline> &paths, const Environment &environment)
  {
    paths_ = paths;
    sort_on_increasing_shortest_path_length(paths_);
    make_time_schedule(paths_,environment);
  }

  void Scheduler::print_paths()
  {
    for (int i = 0; i < paths_.size(); ++i)
    {
      std::cout << "length " << i << ": " << paths_[i].length() << std::endl;
      for (int j = 0; j < paths_[i].size(); ++j)
      {
        std::cout << paths_[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }

  bool sort_comparator(Polyline pl1, Polyline pl2) {return pl1.length() < pl2.length();}

  void Scheduler::sort_on_increasing_shortest_path_length(std::vector<Polyline> &paths)
  {
    std::vector<Polyline> result;
    std::sort( paths.begin(), paths.end(), sort_comparator);
  }

  void Scheduler::schedule(std::vector<std::vector<int>> &time_schedule, const Polyline &path)
  {
    int path_size = path.size();
    Point current_point;
    Point next_point;
    int i=0;
    int t = 0;
    while (i < path_size-1)
    {
      current_point = path[i];
      next_point = path[i+1];
    }
  }

  void Scheduler::make_time_schedule(const std::vector<Polyline> &paths, const Environment &environment)
  {
    std::vector<std::vector<int>> time_schedule;
    
    for (int i = 0; i < environment.n(); ++i)
    {
      time_schedule.push_back(std::vector<int>());
    }

    std::vector<Polyline> prioritization_queue = paths;
    Polyline current_path;

    while (!prioritization_queue.empty())
    {
      current_path = prioritization_queue.back();
      prioritization_queue.pop_back();

      schedule(time_schedule, current_path);
    } 
  }


/*
time_scheme[][]

while !prioritization_queue.empty()
  
  tug = prioritization_queue.pop_front()

  add_to_scheme(tug, time_scheme)




function add_to_scheme(tug, time_scheme)
  t = 0
  for sub_path in tug.path
    while 1
      if sub_path(t).is_available(sub_path_length)
        time_scheme[sub_path, t] = tug
        break
      */


  
  /*Point Scheduler::position_at_time(time_t t, const Boat tug, const Polyline &path, float speed)
  { //speed [m/s] 
    time_t now = time(NULL);
    double seconds = difftime(t, now);
    //assert t is after now
    double time_sum = 0;
    int i = 0;
    while (time_sum < seconds)
    {
      time_sum += eucledian_distance(path[i], path[i+1]) / speed;
    }
    
  }

  double Scheduler::eucledian_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }

*/
}