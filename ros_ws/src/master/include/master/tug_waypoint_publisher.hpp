#ifndef WAYPOINT_PUBLISHER_H
#define WAYPOINT_PUBLISHER_H

#include <ros/ros.h>
#include "master/WaypointAvailable.h"
#include "master/AvoidShipCollision.h"
#include "master/Path.h"
#include "master/ClearWaypoint.h"
#include "master/BoatPose.h"
#include "master/Waypoint.h"
#include "std_msgs/UInt8.h"



#include <memory>

namespace Tug
{
  class Waypoint_publisher
  {
  public:
    Waypoint_publisher(int id, double scale);

    void set_path(const master::Path::ConstPtr &msg);
    void update_position(const master::BoatPose::ConstPtr &msg);
    void set_id(int id){id_ = id;};
    int id() const {return id_;};

    master::BoatPose get_position(){return newest_pose_;}
    void wait_at_current_point();

    std::vector<master::Waypoint> get_path() const {return path_;}
    master::Waypoint get_current_waypoint();
    master::Waypoint get_goal(){return path_.back();}
    void update_only_goal(master::Waypoint goal);
    bool is_waypoint_available(const master::Waypoint &pt);

    int no_waypoints_left(){return path_.size() - current_waypoint_index_;};
    void callback_update_goal(const master::Waypoint::ConstPtr &msg);
  private:
    int id_=-1;
    std::vector<master::Waypoint> path_;
    int current_waypoint_index_ = 0; 
    bool go_to_next_waypoint();

    double scale_;
    double acceptance_radius;

    master::BoatPose newest_pose_;
    int order_id_;


    ros::NodeHandle node_;
    ros::Publisher wayp_pub;
    ros::Publisher arrival_pub;

    ros::ServiceClient client_is_avail;
    ros::ServiceClient client_avoid_ship;

  };
}

#endif //WAYPOINT_PUBLISHER_H