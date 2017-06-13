#include "tug_communicator.hpp"

namespace Tug
{
  Communicator::Communicator( Environment &environment, double scale, 
                              double accept_waypoint_radius)
  {
    scale_ = scale;
    accept_waypoint_radius_ = accept_waypoint_radius;
    environment_tug_ = environment;
    tug_arrived_pub = node_.advertise<tugboat_control::ClearWaypoint>("clearWaypoint", 20);
    path_pub = node_.advertise<tugboat_control::Path>("paths", 20);
    goal_point_pub = node_.advertise<tugboat_control::Waypoint>("goal_point_updater",20);
    assigner_ptr_ = std::make_shared<Assign_paths>(environment);
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
  bool Communicator::tug_is_under_my_control(int id)
  {
    for (int i = 0; i < tugs_under_my_control_.size(); ++i)
    {
      if (id==tugs_under_my_control_[i])
      {
        return true;
      }
    }
    return false;
  }

  void Communicator::polyline_to_path_msg(const Tug::Polyline &path, 
                                          int tug_id, 
                                          int order_id, 
                                          tugboat_control::Path &path_msg)
  {
    path_msg.orderID = order_id;
    path_msg.tugID = tug_id;

    for (int i = 0; i < path.size(); ++i)
    {
      tugboat_control::Waypoint wp; 
      wp.x = path[i].x(); 
      wp.y = path[i].y();
      wp.v = TUG_SPEED;
      wp.ID = tug_id;
      path_msg.data.push_back(wp);
    }
  }


  void Communicator::replan_route_for_one_boat(int order_id, const Tug::Point &newGoal)
  {
    try
    {
      int tug_id = msg_and_tug_.at(order_id);

      if (!tug_is_under_my_control(tug_id))
      {
        return;
      }
      Tug::Point start = tugs_.at(tug_id).get_position();
      Tug::Point finish = newGoal;
      Tug::Polyline spath;
      Tug::Shortest_path sp_node(environment_tug_, start, finish, spath);

      tugboat_control::Path path_msg;
      polyline_to_path_msg(spath, tug_id, order_id, path_msg);

      if (spath.size() > 0)
      {
        path_pub.publish(path_msg);
      }
    }
    catch(const std::out_of_range &oor)
    {
      ROS_WARN("out of range in function replan_route_for_one_boat");
    }
  }

  void Communicator::replan()
  {
    ROS_INFO("replan");


    std::vector<Tug::Boat> tugs_to_plan_for;
    for (int i = 0; i < tugs_under_my_control_.size(); ++i)
    {
      try
      {
        tugs_to_plan_for.push_back(tugs_.at(tugs_under_my_control_[i]));
      }
      catch(const std::out_of_range &err){}
    }

    if (tugs_to_plan_for.size() == 0){return;}
    
    assigner_ptr_->assign_on_combined_shortest_path(tugs_to_plan_for, 
                                                    end_points_, 
                                                    environment_tug_); 
    for (int i = 0; i < tugs_to_plan_for.size(); ++i)
    {
      Polyline path = tugs_to_plan_for[i].get_path();

      if (path.size() > 0)
      {
        tugboat_control::Path path_msg;
        polyline_to_path_msg(path, tugs_to_plan_for[i].id(), 
                             find_order_id(path.back()), path_msg);

        try
        {
          msg_and_tug_.at(path_msg.orderID) = path_msg.tugID;
        }
        catch(const std::out_of_range &oor)
        {
          msg_and_tug_.insert(std::pair<int,int>(path_msg.orderID,path_msg.tugID));
        }

        path_pub.publish(path_msg);
      }
      else
      {
        ROS_WARN("Could not find possible route");
      }
    }
  }


  void Communicator::callback_waypoint(const tugboat_control::Waypoint::ConstPtr& msg)
  { 
    if (tugs_under_my_control_.size() == 0 || tugs_.size() == 0)
    {
      ROS_WARN("No tugs under my control");
      return;
    }
    for (std::map<int, Boat>::iterator i = tugs_.begin(); i != tugs_.end(); ++i)
    {
      Point pt(i->second.get_position());
      if (pt.x()==-1 && pt.y()==-1)
      {
        ROS_WARN("Positions of tugs not set");
        return;
      }
    }

    Tug::Point pt(msg->x*scale_, msg->y*scale_, environment_tug_);
    int order_id = msg->ID;

    try
    {
      if (end_points_.at(order_id).x() == pt.x() && end_points_.at(order_id).y() == pt.y())
      {
        return;
      }
      if (pow(end_points_.at(order_id).x() - pt.x(), 2) + 
          pow(end_points_.at(order_id).y() - pt.y(), 2) < 0.05*scale_)
      {
        end_points_.at(order_id) = pt;
        replan_route_for_one_boat(order_id, pt);
      }
    }
    catch(const std::out_of_range &oor)
    {
      for (std::map<int, Boat>::iterator i = tugs_.begin(); i != tugs_.end(); ++i)
      {
        Point pos = i->second.get_position();
        if (sqrt(pow(pos.x() - pt.x(), 2) + pow(pos.y() - pt.y(), 2)) <
                                         accept_waypoint_radius_*scale_)
        {
          ROS_INFO("A tug is already at goal");
          return;
        }
      }
      //If waypoint is new
      end_points_.insert(std::pair<int, Tug::Point>(order_id, pt));
      replan();

    }
  }

  void Communicator::remove_tug_from_control(int tug_id)
  {
    for (std::vector<int>::iterator i = tugs_under_my_control_.begin(); 
                                    i != tugs_under_my_control_.end(); 
                                    ++i)
    {
      if (*i == tug_id)
      {
        try
        {
          tugs_under_my_control_.erase(i);
          return;
        }
        catch(...){return;}
      }
    }
  }


  int Communicator::find_order_id(const Tug::Point &pt)
  {
    for (std::map<int,Tug::Point>::iterator i = end_points_.begin(); 
                                            i != end_points_.end(); 
                                            ++i)
    {
      if (i->second == pt)
      {
        return i->first;
      }
    }
    return -1;
  }

  void Communicator::remove_end_point_from_planner(
                            const tugboat_control::ClearWaypoint::ConstPtr &msg)
  {
    try
    {
      end_points_.erase(msg->orderID);
      order_ready_to_publish.push_back(*msg);
      remove_tug_from_control(msg->tugID);

      if (end_points_.size() == 0 || order_ready_to_publish.size() > 1)
      {
        for (int i = 0; i < order_ready_to_publish.size(); ++i)
        {
          tug_arrived_pub.publish(order_ready_to_publish[i]);
        }
        order_ready_to_publish.clear();
      }
    }
    catch(...)
    {
      ROS_WARN("Could not erase end point with order id %d", msg->orderID);
    }
  }

  void Communicator::callback_boat_pose(const tugboat_control::BoatPose::ConstPtr& msg)
  {
    if (tugs_.size() == 0)
    {
      return;
    }

    int id = msg->ID;

    Tug::Point pt(msg->x*scale_, msg->y*scale_, environment_tug_);
    try
    {
      tugs_.at(id).update_position(pt);
    }
    catch(const std::out_of_range &oor)
    {
      ROS_WARN("Tug %d has not started yet", id);
      return;
    }
  }

  void Communicator::callback_available_tugs(
                        const std_msgs::UInt8MultiArray::ConstPtr &msg)
  {
    tugs_under_my_control_.clear();
    for (int i = 0; i < msg->data.size(); ++i)
    {
      tugs_under_my_control_.push_back(msg->data[i]);
    }
  }

  void Communicator::add_new_tug(int id)
  {
    Boat tug;
    tug.set_id(id);
    tugs_.insert(std::pair<int, Tug::Boat>(id, std::move(tug)));
    ROS_INFO("Added tug %d", id);
  }

  bool Communicator::tug_id_already_in_system(int id)
  {
    try
    {
      tugs_.at(id);
      return true;
    }
    catch(const std::out_of_range &oor)
    {
      return false;
    }
  }

  void Communicator::callback_new_tug(const std_msgs::UInt8::ConstPtr &msg)
  {
    if (tug_id_already_in_system(msg->data))
    {
      return;
    }
    //startup message
    add_new_tug(msg->data);
  }

}