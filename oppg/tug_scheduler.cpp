#include "include/tug_scheduler.hpp"

namespace Tug
{
  Scheduler::Scheduler(std::vector<Polyline> &paths, const Environment &environment)
  {
    //paths = paths;
    sort_on_increasing_shortest_path_length(paths);
    std::vector<std::vector<int>> time_schedule;
    make_time_schedule(time_schedule, paths,environment);
    print_schedule(time_schedule);
  }

  void Scheduler::print_paths(std::vector<Polyline> &paths)
  {
    for (int i = 0; i < paths.size(); ++i)
    {
     // std::cout << "length " << i << ": " << paths[i].length() << std::endl;
      for (int j = 0; j < paths[i].size(); ++j)
      {
        std::cout << paths[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
  void Scheduler::print_schedule(std::vector<std::vector<int>> &time_schedule)
  {
    for (int i = 0; i < time_schedule.size(); ++i)
    {
      std::cout << "Point " << i << std::endl;
      for (int j = 0; j < time_schedule[i].size(); ++j)
      {
        std::cout << time_schedule[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }

  bool sort_comparator(Polyline pl1, Polyline pl2) {return pl1.length() < pl2.length();}

  void Scheduler::sort_on_increasing_shortest_path_length( std::vector<Polyline> &paths)
  {
    std::vector<Polyline> result;
    std::sort( paths.begin(), paths.end(), sort_comparator);
  }

  bool Scheduler::is_available(std::vector<int> &time_schedule_point, int t)
  {
    if (t >= time_schedule_point.size() || time_schedule_point[t] == 0)
    {
      return true;
    }
    return false;
  }

  void Scheduler::set_tug(std::vector<int> &time_schedule_point, int id, int t)
  {
    if (t >= time_schedule_point.size())
    {
      int no_extra_elements = t - time_schedule_point.size() + 1;
      std::vector<int> zeros(no_extra_elements, 0);
      time_schedule_point.insert(time_schedule_point.end(), zeros.begin(), zeros.end());
    }
    time_schedule_point[t] = id;
  }



  void Scheduler::schedule(std::vector<std::vector<int>> &time_schedule, Polyline &path, const Boat &tug)
  {
    int t_tot = 0;
    for (int i = 0; i < path.size()-2; ++i) //last point has path.id = -1
    {
      int length = round(eucledian_distance(path[i], path[i+1]));
      bool planned = false;
      while (!planned)
      {
        int t = t_tot+length;
        //std::cout << path[i+1].id() << std::endl;
        if (!is_available(time_schedule[path[i+1].id()], t))
        {
          t_tot++;
        }
        else
        {
          set_tug(time_schedule[path[i+1].id()], tug.id(), t);
          planned = true;
        }
      }
    }
  }
    
  void Scheduler::make_time_schedule(std::vector<std::vector<int>> &time_schedule, std::vector<Polyline> &paths, const Environment &environment)
  {
    
    for (int i = 0; i < environment.n(); ++i)
    {
      time_schedule.push_back(std::vector<int>());
    }

    for (int i = paths.size()-1; i >= 0; --i)
    {
      Boat tug(7.0,Point(20,20,-1));
      tug.set_id(i);

      schedule(time_schedule, paths[i], tug);
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
*/
  double Scheduler::eucledian_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }


}