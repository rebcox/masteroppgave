#include "include/tug_scheduler.hpp"

namespace Tug
{
  Scheduler::Scheduler(std::vector<Boat> &tugs, const Environment &environment)
  {
    sort_on_increasing_shortest_path_length(tugs);
    make_time_schedule(time_schedule_, tugs, environment);
  }

  void Scheduler::print_paths(std::vector<Boat> &tugs)
  {
    for (int i = 0; i < tugs.size(); ++i)
    {
     // std::cout << "length " << i << ": " << tugs[i].length() << std::endl;
      Polyline path = tugs[i].get_path();
      std::cout << "Tug " << tugs[i].id() << ": ";
      for (int j = 0; j < path.size(); ++j)
      {

        std::cout << path[j] << " ";
      }
      std::cout << std::endl;
    }
  }
  void Scheduler::print_schedule()
  {
    for (int i = 0; i < time_schedule_.size(); ++i)
    {
      std::cout << "Point " << i << std::endl;
      for (int j = 0; j < time_schedule_[i].size(); ++j)
      {
        std::cout << time_schedule_[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }

  bool sort_comparator(Polyline pl1, Polyline pl2) {return pl1.length() < pl2.length();}

  void Scheduler::sort_on_increasing_shortest_path_length( std::vector<Polyline> &paths)
  {
    std::sort( paths.begin(), paths.end(), sort_comparator);
  }

  bool sort_comparator1(Boat tug1, Boat tug2) {return tug1.get_path().length() < tug2.get_path().length();}

  void Scheduler::sort_on_increasing_shortest_path_length( std::vector<Boat> &tugs)
  {
    std::sort( tugs.begin(), tugs.end(), sort_comparator1);
  }

  bool Scheduler::time_schedule_is_zero(std::vector<int> &time_schedule, int t, int duration)
  {
    for (int i = t; i <= t+duration; ++i)
    {
      if (time_schedule[i] != 0)
      {
        return false;
      }
    }
    return true;
  }

  bool Scheduler::is_available(std::vector<std::vector<int>> &time_schedule, 
                                std::vector<Waypoint> &waypoints,
                                int id,
                                int t,
                                int duration)
  {
    if (t + duration >= time_schedule[id].size() || time_schedule_is_zero(time_schedule[id], t, duration)) // time_schedule[id][t] == 0)
    {
      for (int i = 0; i < waypoints[id].n_points_within_range(); ++i)
      {
        int point_id = waypoints[id].point_within_range(i)->id();
        if (!(t >= time_schedule[point_id].size() || time_schedule_is_zero(time_schedule[point_id], t, duration) ))//time_schedule[point_id][t] == 0))
        {
          return false;
        }
      }
      return true;
    }
    return false;
  }

  void Scheduler::set_tug(std::vector<int> &time_schedule_point, int id, int t, int duration)
  {
    if (t+duration >= time_schedule_point.size())
    {
      int no_extra_elements = t + duration - time_schedule_point.size();
      std::vector<int> zeros(no_extra_elements, 0);
      time_schedule_point.insert(time_schedule_point.end(), zeros.begin(), zeros.end());
    }
    for (int i = 0; i < duration; ++i)
    {
      time_schedule_point[t+i] = id;
    }
    
  }



  void Scheduler::schedule(std::vector<std::vector<int>> &time_schedule, const Boat &tug, std::vector<Waypoint> &waypoints)
  {
    double speed = tug.get_top_speed();
    int duration_in_waypoint = 5;
    int t_tot = 0;
    Polyline path = tug.get_path();
    for (int i = 0; i < path.size()-2; ++i) //last point has path.id = -1
    {
      int length = round(eucledian_distance(path[i], path[i+1]));
      bool planned = false;
      while (!planned)
      {
        int t = t_tot+length/speed;
        if (!is_available(time_schedule, waypoints, path[i+1].id(), t, duration_in_waypoint))
        {
          t_tot++;
        }
        else
        {
          set_tug(time_schedule[path[i+1].id()], tug.id(), t, duration_in_waypoint);
          planned = true;
        }
      }
    }
  }
    
  void Scheduler::make_time_schedule(std::vector<std::vector<int>> &time_schedule,
                                     std::vector<Boat> &tugs, const Environment &environment)
  {
    std::vector<Waypoint> waypoints;
    for (int i = 0; i < environment.n(); ++i)
    {
      time_schedule.push_back(std::vector<int>());
      //waypoints.push_back(Waypoint(environment(i),7,environment,waypoints,i));
      Point pt = environment(i);
      waypoints.push_back(Waypoint(pt,pt.id(), 7,waypoints));

      std::cout << "point " << pt.id() << " " << waypoints[i] << std::endl;
    }

    for (int i = tugs.size()-1; i >= 0; --i)
    {
      schedule(time_schedule, tugs[i], waypoints);
    }     
  }


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