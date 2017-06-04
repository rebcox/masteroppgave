#include "tug_communicator.hpp"

namespace Tug
{
  Communicator::Communicator( Environment &environment, double scale, double accept_waypoint_radius)
  {
    scale_ = scale;
    accept_waypoint_radius_ = accept_waypoint_radius;
    environment_tug_ = environment;
    tug_arrived_pub = node_.advertise<master::ClearWaypoint>("clearWaypoint", 20);
    path_pub = node_.advertise<master::Path>("paths", 20);
    goal_point_pub = node_.advertise<master::Waypoint>("goal_point_updater",20);
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
    
    assigner.assign_on_combined_shortest_path(tugs_to_plan_for, end_points_, environment_tug_); 
    
    for (int i = 0; i < tugs_to_plan_for.size(); ++i)
    {
      Polyline path = tugs_to_plan_for[i].get_path();
      master::Path path_msg;

      for (int j = 0; j < path.size(); ++j)
      {

        master::Waypoint wp; 
        wp.x = path[j].x(); 
        wp.y = path[j].y();
        wp.v = 10;
        wp.ID = tugs_to_plan_for[i].id();
        path_msg.data.push_back(wp);
      }
      if (path.size() > 0)
      {
        path_msg.orderID = find_order_id(path.back());
        path_msg.tugID = tugs_to_plan_for[i].id();
        path_pub.publish(path_msg);
      }
    }
  }


  void Communicator::callback_waypoint(const master::Waypoint::ConstPtr& msg)
  { 
    //ROS_INFO("callback_waypoint");

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
    int msg_id = msg->ID;

    try
    {
      //If waypoint with msg_id is updated
      if (end_points_.at(msg_id) != pt)
      {
        end_points_.at(msg_id) = pt;
        goal_point_pub.publish(msg);
      }
    }
    catch(const std::out_of_range &oor)
    {
      for (std::map<int, Boat>::iterator i = tugs_.begin(); i != tugs_.end(); ++i)
      {
        Point pos = i->second.get_position();
        if (sqrt(pow(pos.x() - pt.x(), 2) + pow(pos.y() - pt.y(), 2)) < accept_waypoint_radius_*scale_)
        {
          return;
        }
      }
      //If waypoint is new
      end_points_.insert(std::pair<int, Tug::Point>(msg_id, pt));
      replan();

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

  void Communicator::remove_end_point_from_planner(const master::ClearWaypoint::ConstPtr &msg)
  {
    try
    {
      end_points_.erase(msg->orderID);
      order_ready_to_publish.push_back(*msg);
      if (end_points_.size() == 0)
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

  void Communicator::callback_boat_pose(const master::BoatPose::ConstPtr& msg)
  {
    //ROS_INFO("callback_boat_pose");
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

  void Communicator::callback_available_tugs(const std_msgs::UInt8MultiArray::ConstPtr &msg)
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
    ROS_INFO("....................callback_new_tug....................");
    //startup message
    add_new_tug(msg->data);
  }

}