
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

#include "tug_point.hpp"
#include "tug_polyline.hpp"
#include "tug_environment.hpp"
#include <memory>
//#include "tug_switching_point_decision.hpp"

namespace Tug
{
  class Boat
  {
  public:
    Boat(double radius, const Point &position, Environment *environment);
    Boat(){};
    void set_top_speed(double top_speed){top_speed_ = top_speed;}
    double get_top_speed() const{return top_speed_;}
    void set_path(const Polyline &path);
    double get_radius(){return radius_;};
    Point  get_position();
    void update_position(Point &position, bool &waypoint_updated_flag, bool &arrived_at_goal_flag, double radius);
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
    Polyline get_path() const {return path_;}
    //std::shared_ptr<Point> get_current_waypoint();
    Point get_current_waypoint();
    Point get_goal(){return path_.back();}

    int no_waypoints_left(){return path_.size() - current_waypoint_index_;};
    const std::vector<int> is_held_by() const {return is_held_by_;}
    const std::vector<int> is_holding() const {return is_holding_;}

    bool remove_tug_holding_this(int id);
    bool remove_tug_held_by_this(int id);

    bool set_tug_holding_this(int id);
    bool set_tug_held_by_this(int id);

    void clear_tug();

    bool is_free_to_move();

  private:
    double radius_;
    double top_speed_ = 1; //pixels per time unit
    Point position_;
    Environment *environment_;
    int id_=-1;
    Polyline path_;
    int current_waypoint_index_ = 0; 
    bool go_to_next_waypoint();
    std::vector<int> is_held_by_;
    std::vector<int> is_holding_;
  };
}

#endif //TUG_BOAT_H

