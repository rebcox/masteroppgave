#include "tug_communicator.hpp"

namespace Tug
{
  Communicator::Communicator(std::map<int,Boat> &tugs, Environment &environment, double scale)
  {
    for (std::map<int,Boat>::iterator i = tugs.begin(); i != tugs.end(); ++i)
    {
      add_new_tug(i->second, i->first);
    }
    
    scale_ = scale;
    route_around_ship_ = Tug::Route_around_ship(0,2,4);
    environment_tug_ = environment;
    waypoint_pub = node_.advertise<master::Waypoint>("waypoint", 20);
    tug_arrived_pub = node_.advertise<master::ClearWaypoint>("clearWaypoint", 20);
  }

  float Communicator::eucledian_distance(const Tug::Point &point1, const Tug::Point &point2)
  {
    return sqrt(pow((point1.x() - point2.x())/scale_, 2) + pow((point1.y() - point2.y())/scale_, 2));
  }

  void Communicator::print_path(Tug::Polyline path)
  {
    std::stringstream path_string;
    for (int i = 0; i < path.size(); ++i)
    {
      path_string << path[i];
      if (i < path.size()-1)
      {
        path_string << " - ";
      }
    }
    ROS_INFO("Shortest path: %s", path_string.str().c_str());
  }

  void Communicator::replan()
  {
    Tug::Assign_paths assigner;
    //update_tug_positions();

    std::vector<Tug::Boat> tugs_to_plan_for;
    for (int i = 0; i < tugs_under_my_control_.size(); ++i)
    {
      tugs_to_plan_for.push_back(tugs_.at(tugs_under_my_control_[i]));
    }

    for (int i = 0; i < tugs_to_plan_for.size(); ++i)
    {
      ROS_INFO("planning for tug %d", tugs_to_plan_for[i].id());
    }
    for (std::map<int, Tug::Point>::iterator i = end_points_.begin(); i != end_points_.end(); ++i)
    {
      ROS_INFO("planning for end point (%f, %f)", i->second.x(), i->second.y());
    }
    assigner.assign_on_combined_shortest_path(tugs_to_plan_for, end_points_, environment_tug_); 
    
    for (int i = 0; i < tugs_to_plan_for.size(); ++i)
    {
      tugs_.at(tugs_to_plan_for[i].id()).set_path(tugs_to_plan_for[i].get_path());
    }

    for (std::map<int, Tug::Boat>::iterator tug = tugs_.begin(); tug != tugs_.end(); ++tug)
    {
      ROS_INFO("Tug %d: ", tug->first);
      print_path(tug->second.get_path());
    }
  }

  void Communicator::remove_end_point_from_planner(int point_id)
  {
    end_points_.erase(point_id);
  }

  void Communicator::set_holding_tug(int holding_id, int held_id)
  {
    tugs_.at(holding_id).set_tug_held_by_this(held_id);
    tugs_.at(held_id).set_tug_holding_this(holding_id);
  }

  void Communicator::remove_holding_tug(int holding_id, int held_id)
  {
    tugs_.at(holding_id).remove_tug_held_by_this(held_id);
    tugs_.at(held_id).remove_tug_holding_this(holding_id);
  }

  void Communicator::publish_new_waypoint(const Tug::Point pt_cur, int tug_id)
  {
    master::Waypoint waypoint;
    waypoint.ID = tug_id;
    waypoint.x = pt_cur.x(); waypoint.y = pt_cur.y();
    waypoint.v = 10;

    bool another_tug_in_the_way = false;

    //Check for tugs in the way of point
    for (std::map<int, Tug::Point>::iterator pt = current_waypoints_.begin(); pt != current_waypoints_.end(); ++pt)
    {
      if (pt->first == tug_id)
       {
         continue;
       } 
       //if a current waypoint is within a certain range of the point to be published, publish it with v=0
       if (pt->second == pt_cur ||  eucledian_distance(pt->second, pt_cur) < 0.01*scale_) //TODO: hardkoda range
       {
          waypoint.v = 0;
          ROS_WARN("tug %d is held by tug %d", tug_id, pt->first);
          set_holding_tug(pt->first, tug_id);
          another_tug_in_the_way = true;
       }
    }

    ROS_INFO("Tug %d: Published new point (%f, %f)", tug_id, pt_cur.x(), pt_cur.y());
    if (waypoint.v > 0)
    {
      waypoint_pub.publish(waypoint);
    }
    
    //if new point with speed > 0 will be published,
    //check if the tug had other tugs dependent on it
    if (!another_tug_in_the_way)
    {
      //set current_waypoints_
      current_waypoints_.at(tug_id) = pt_cur;
      //Looping through all tugs the new published point were holding
      std::vector<int> current_tug_is_holding = tugs_.at(tug_id).is_holding();

      for (int i = 0; i < current_tug_is_holding.size(); ++i)
      {
        remove_holding_tug(tug_id, current_tug_is_holding[i]);
        publish_new_waypoint(tugs_.at(current_tug_is_holding[i]).get_current_waypoint(), 
                              current_tug_is_holding[i]);
      }
    }
  }

  int Communicator::find_order_id(const Tug::Point &pt)
  {
    for (std::map<int,Tug::Point>::iterator i = end_points_.begin(); i != end_points_.end(); ++i)
    {
      if (i->second == pt)
      {
        return i->first;
      }
    }
    return -1;
  }

  void Communicator::remove_tug_from_control(int tug_id)
  {
    for (std::vector<int>::iterator i = tugs_under_my_control_.begin(); i != tugs_under_my_control_.end(); ++i)
    {
      if (*i == tug_id)
      {
        tugs_under_my_control_.erase(i);
        break;
      }
    }
  }

  void Communicator::publish_arrived_tug(int tug_id)
  {
    //ROS_INFO("Tug %d has arrived", tug_id);
    master::ClearWaypoint clear_point;
    clear_point.tugID = tug_id;

    int order_id = find_order_id(current_waypoints_.at(tug_id));
    if (order_id == -1)
    {
      ROS_ERROR("Could not find order id of point (%f, %f)", 
                 current_waypoints_.at(tug_id).x(),
                 current_waypoints_.at(tug_id).y());
    }
    else
    {
      ROS_INFO("erased from endpoint: (%f, %f)", end_points_.at(order_id).x(), end_points_.at(order_id).y());
      end_points_.erase(order_id);
      current_waypoints_.erase(tug_id);
      remove_tug_from_control(tug_id);
      tugs_.at(tug_id).clear_tug();
    }

    clear_point.orderID = order_id;
    tug_arrived_pub.publish(clear_point);
  }

  bool Communicator::is_under_my_control(int id)
  {
    for (int i = 0; i < tugs_under_my_control_.size(); ++i)
    {
      if (tugs_under_my_control_[i] == id)
      {
        return true;
      }
    }
    return false;
  }

  void Communicator::add_new_tug(Tug::Boat &tug, int id)
  {
    tug.set_id(id);
    tugs_.insert(std::pair<int, Tug::Boat>(id, tug));
    current_waypoints_.insert(std::pair<int, Tug::Point>(id, tug.get_position()));
  }

  void Communicator::callback_waypoint(const master::Waypoint::ConstPtr& msg)
  { 
    if (tugs_under_my_control_.size() == 0)
    {
      return;
    }
    Tug::Point pt(msg->x*scale_, msg->y*scale_, environment_tug_);
    int msg_id = msg->ID;

    try
    {
      //If waypoint with msg_id is updated
      if (end_points_.at(msg_id) != pt)
      {
        end_points_.at(msg_id) = pt;
        replan();
      }
    }
    catch(const std::out_of_range &oor)
    {
      //If waypoint is new
      end_points_.insert(std::pair<int, Tug::Point>(msg_id, pt));
      replan();
    }
  }

  void Communicator::callback_boat_pose(const master::BoatPose::ConstPtr& msg)
  {
    if (end_points_.size() == 0)
    {
      return;
    }

    int id = msg->ID;

    if (!is_under_my_control(id))
    {
      return;
    }

    Tug::Point pt(msg->x*scale_, msg->y*scale_, environment_tug_);
    bool new_waypoint_set;
    bool arrived_at_goal;

    tugs_.at(id).update_position(pt, new_waypoint_set, arrived_at_goal, 0.3*scale_); //TODO: hardkoda

    //Tug::Point* pt_cur = tugs_.at(id).get_current_waypoint();
    //std::shared_ptr<Tug::Point> 
    Tug::Point pt_cur = tugs_.at(id).get_current_waypoint();
    //Not active
    //if (!pt_cur)
    if(pt_cur.x()==-1 && pt_cur.y()==-1)
    {
      std::cout << "not active" << std::endl;
      return;
    }

    if (new_waypoint_set)
    {
      publish_new_waypoint(pt_cur, id);
      //Heading towards second to last waypoint
      /*if(tugs_.at(id).no_waypoints_left() == 2)
      {
        Tug::Polyline points_to_go_around_ship = route_around_ship_.best_route(pt_cur, tugs_.at(id).get_goal(), environment_tug_);
        
        if (points_to_go_around_ship.size() > 0)
        {
          points_to_go_around_ship.push_front(pt_cur);
          points_to_go_around_ship.push_back(tugs_.at(id).get_goal());
          tugs_.at(id).set_path(points_to_go_around_ship);
        }
      }*/
    }
    else if(arrived_at_goal)
    {
      master::Waypoint waypoint;
      waypoint.ID = id;
      waypoint.x = pt_cur.x();
      waypoint.y = pt_cur.y();
      waypoint.v = 0;
      waypoint_pub.publish(waypoint);
      publish_arrived_tug(id);
    }
    else
    {
      /*master::Waypoint waypoint;
      waypoint.ID = id;
      waypoint.x = pt_cur.x();
      waypoint.y = pt_cur.y();

      if (tugs_.at(id).is_free_to_move())
      {
        waypoint.v = 10;
        waypoint_pub.publish(waypoint);
      }
      else
      {
        waypoint.v = 0;
      }
      */
      //  ROS_INFO("Tug %d: Publ point (%f, %f) v: %f", id, pt_cur->x(), pt_cur->y(), waypoint.v);
    }
  }

  void Communicator::callback_ship_pose(const master::BoatPose::ConstPtr &msg)
  {
    Tug::Point mid_pt(msg->x*scale_, msg->y*scale_, environment_tug_);
    route_around_ship_.move(mid_pt, msg->o);
  }

  void Communicator::callback_available_tugs(const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    tugs_under_my_control_.clear();
    for (int i = 0; i < msg->data.size(); ++i)
    {
      tugs_under_my_control_.push_back(msg->data[i]);
    }
  }

}