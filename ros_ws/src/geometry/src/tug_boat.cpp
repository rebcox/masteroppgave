#include "tug_boat.hpp"
#include <stdexcept>
#include "ros/ros.h"
namespace Tug
{
  Boat::Boat(double radius, const Point &position, Environment *environment)
  {
    if (radius > 0)
    {
      radius_ = radius;
      position_ = position;
      environment_ = environment;
    }
    else
    {
      throw std::invalid_argument( "received negative value of length or width" );
    }
  }
  
  Point Boat::get_position()
  {
    return position_;
  }

  void Boat::update_position(Point &position, bool &waypoint_updated_flag, bool &arrived_at_goal_flag, double radius)
  {
    position_ = position;

    waypoint_updated_flag = false;
    arrived_at_goal_flag = false;
    try
    {
      Point *current_wp;
      if (current_waypoint_index_ >= path_.size())
      {
        return;
      }
      current_wp = &path_[current_waypoint_index_];

      //if (current_waypoint_index_ == 0 || (sqrt(pow(position.x() - current_wp->x(), 2) + pow(position.y() - current_wp->y(), 2)) < radius))
      //{
        //waypoint_updated_flag = true;
      //}

      //Check if position is within radius of current waypoint
      if(sqrt(pow(position.x() - current_wp->x(), 2) + pow(position.y() - current_wp->y(), 2)) < radius)
      {
        waypoint_updated_flag = go_to_next_waypoint();
        if (!waypoint_updated_flag)
        {
          arrived_at_goal_flag = true;
        }
      }
    }
    catch(...)
    {
      std::cerr << "Error in Boat::update_position" << std::endl;
      return;
    }
    

  }

  void Boat::set_path(const Polyline &path)
  {
    path_.clear();
    path_ = path;
    current_waypoint_index_= 0;
    //go_to_next_waypoint();    
  }

  bool Boat::go_to_next_waypoint()
  {
    ++current_waypoint_index_;
    if (current_waypoint_index_ >= path_.size())
    {
      --current_waypoint_index_;
      return false;
    }
    else
    {
      return true;
    }
  }

  //std::shared_ptr<Point> Boat::get_current_waypoint()
  Point Boat::get_current_waypoint()
  {
    //shared_ptr<Song> p1(song1);

    if(current_waypoint_index_ < path_.size())
    {

      //std::shared_ptr<Point> ptr(&path_[current_waypoint_index_]);
      //return ptr;
      return path_[current_waypoint_index_];
    }
    else
    {
      return Point(-1,-1,-1);
      //return nullptr;
    }
  }

  bool Boat::remove_tug_holding_this(int id)
  {
    for (std::vector<int>::iterator tug = is_held_by_.begin(); tug != is_held_by_.end(); ++tug)
    {
      if (*tug == id)
      {
        is_held_by_.erase(tug);
        return true;
      }
    }
    return false;
  }
  
  bool Boat::remove_tug_held_by_this(int id)
  {
    for (std::vector<int>::iterator tug = is_holding_.begin(); tug != is_holding_.end(); ++tug)
    {
      if (*tug == id)
      {
        is_holding_.erase(tug);
        return true;
      }
    }
    return false;
  }

  bool Boat::set_tug_holding_this(int id)
  {
    for (int i = 0; i < is_held_by_.size(); ++i)
    {
      if (is_held_by_[i] == id)
      {
        return false;
      }
    }
    is_held_by_.push_back(id);
    return true;
  }


  bool Boat::set_tug_held_by_this(int id)
  {
    for (int i = 0; i < is_holding_.size(); ++i)
    {
      if (is_holding_[i] == id)
      {
        return false;
      }
    }
    is_holding_.push_back(id);
    return true;
  }
  
  bool Boat::is_free_to_move()
  {
    if (is_held_by_.size() == 0)
    {
      return true;
    }
    return false;
  }

  void Boat::clear_tug()
  {
    current_waypoint_index_ = 0;
    is_held_by_.clear();
    is_holding_.clear();
  }

}