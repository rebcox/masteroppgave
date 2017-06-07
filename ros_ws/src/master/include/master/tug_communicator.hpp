#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sstream>
#include <string>
#include <memory>

#include "coordination/tug_assign_paths.hpp"
#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "tugboat_control/BoatPose.h"
#include "tugboat_control/ClearWaypoint.h"
#include "tugboat_control/Waypoint.h"
#include "tugboat_control/Path.h"
#include "std_msgs/UInt8.h"

#include "search/tug_route_around_ship.hpp"
#include "search/tug_shortest_path.hpp"

namespace Tug
{
  class Communicator
  {
  public:
	  Communicator(Environment &environment, double scale, double accept_waypoint_radius);
	  void print_path(Tug::Polyline path);
	  void replan();
  	tugboat_control::Path polyline_to_path_msg(const Tug::Polyline &path, int tug_id, int order_id);
  	  void polyline_to_path_msg(const Tug::Polyline &path, int tug_id, int order_id, tugboat_control::Path &path_msg);

	  void remove_end_point_from_planner(const tugboat_control::ClearWaypoint::ConstPtr &msg);
	  void set_holding_tug(int holding_id, int held_id);
	  void remove_holding_tug(int holding_id, int held_id);
	  void publish_new_waypoint(const Tug::Point pt_cur, int tug_id);
	  int  find_order_id(const Tug::Point &pt);
	  void remove_tug_from_control(int tug_id);
	  void add_new_tug(int id);
    bool tug_id_already_in_system(int id);
    bool tug_is_under_my_control(int id);
      void replan_route_for_one_boat(int msg_id, const Tug::Point &newGoal);
	  void callback_waypoint(const tugboat_control::Waypoint::ConstPtr& msg);
	  void callback_boat_pose(const tugboat_control::BoatPose::ConstPtr& msg);
	  void callback_ship_pose(const tugboat_control::BoatPose::ConstPtr &msg);
	  void callback_available_tugs(const std_msgs::UInt8MultiArray::ConstPtr &msg);
    void callback_new_tug(const std_msgs::UInt8::ConstPtr &msg);


  private:
	  Tug::Environment environment_tug_;
		double accept_waypoint_radius_;
	  double scale_;

  	ros::NodeHandle node_;
	  ros::Publisher tug_arrived_pub;
	  ros::Publisher ship_waypoint_pub;
	  ros::Publisher path_pub;
	  ros::Publisher goal_point_pub;

	  std::map<int, Tug::Boat> tugs_;
	  std::map<int, Tug::Point> end_points_;
	  std::vector<int> tugs_under_my_control_;
	  std::vector<tugboat_control::ClearWaypoint> order_ready_to_publish;
	  std::map<int,int> msg_and_tug_;
 };

}