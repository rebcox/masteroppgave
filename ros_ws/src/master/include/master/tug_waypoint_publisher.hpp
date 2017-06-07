#ifndef WAYPOINT_PUBLISHER_H
#define WAYPOINT_PUBLISHER_H

#include <ros/ros.h>
#include "tugboat_control/WaypointAvailable.h"
#include "tugboat_control/AvoidShipCollision.h"
#include "tugboat_control/Path.h"
#include "tugboat_control/ClearWaypoint.h"
#include "tugboat_control/BoatPose.h"
#include "tugboat_control/Waypoint.h"
#include "std_msgs/UInt8.h"



#include <memory>

namespace Tug
{
  class Waypoint_publisher
  {
  public:
    Waypoint_publisher(int id, double scale);

    void set_path(const tugboat_control::Path::ConstPtr &msg);
    void update_position(const tugboat_control::BoatPose::ConstPtr &msg);
    void set_id(int id){id_ = id;};
    int id() const {return id_;};
    void call_path_around_ship_service(const tugboat_control::Waypoint &start,
                                     const tugboat_control::Waypoint &finish,
                                     std::vector<tugboat_control::Waypoint> &result);

    tugboat_control::BoatPose get_position(){return newest_pose_;}
    void wait_at_current_point();

    std::vector<tugboat_control::Waypoint> get_path() const {return path_;}
    tugboat_control::Waypoint get_current_waypoint();
    tugboat_control::Waypoint get_goal(){return path_.back();}
    void update_only_goal(tugboat_control::Waypoint goal);
    bool is_waypoint_available(const tugboat_control::Waypoint &pt);
    void publish_current_waypoint();
    int no_waypoints_left(){return path_.size() - current_waypoint_index_;};
    void callback_update_goal(const tugboat_control::Waypoint::ConstPtr &msg);
  private:
    int id_=-1;
    std::vector<tugboat_control::Waypoint> path_;
    int current_waypoint_index_ = 0; 
    bool heading_towards_goal = false;
    bool go_to_next_waypoint();

    double scale_;
    double acceptance_radius;

    tugboat_control::BoatPose newest_pose_;
    int order_id_;


    ros::NodeHandle node_;
    ros::Publisher wayp_pub;
    ros::Publisher arrival_pub;

    ros::ServiceClient client_is_avail;
    ros::ServiceClient client_avoid_ship;

  };
}

#endif //WAYPOINT_PUBLISHER_H