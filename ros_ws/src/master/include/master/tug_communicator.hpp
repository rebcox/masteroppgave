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
#include "std_msgs/UInt8.h"
#include "search/tug_route_around_ship.hpp"
#include "search/tug_shortest_path.hpp"

namespace Tug
{
  class Communicator
  {
  public:
	  Communicator(Environment &environment, double scale, double accept_waypoint_radius);
	  float eucledian_distance(const Tug::Point &point1, const Tug::Point &point2);
	  void print_path(Tug::Polyline path);
	  void replan();
	  void remove_end_point_from_planner(int point_id);
	  void set_holding_tug(int holding_id, int held_id);
	  void remove_holding_tug(int holding_id, int held_id);
	  void publish_new_waypoint(const Tug::Point pt_cur, int tug_id);
	  int  find_order_id(const Tug::Point &pt);
	  void remove_tug_from_control(int tug_id);
	  void publish_arrived_tug(int tug_id);
	  bool is_under_my_control(int id);
	  void add_new_tug(Tug::Boat &tug, int id);

	  void callback_waypoint(const tugboat_control::Waypoint::ConstPtr& msg);
	  void callback_boat_pose(const tugboat_control::BoatPose::ConstPtr& msg);
	  void callback_ship_pose(const tugboat_control::BoatPose::ConstPtr &msg);

    bool tug_id_already_in_system(int id);

	  void callback_available_tugs(const std_msgs::UInt8MultiArray::ConstPtr &msg);
    void callback_new_tug(const std_msgs::UInt8::ConstPtr &msg);

  private:
	  std::map<int, Tug::Boat> tugs_;
	  Tug::Environment environment_tug_;
		double accept_waypoint_radius_;
	  std::map<int, Tug::Point> end_points_;
	  std::vector<master::ClearWaypoint> order_ready_to_publish;

  	ros::NodeHandle node_;
	  ros::Publisher waypoint_pub;
	  ros::Publisher tug_arrived_pub;
	  ros::Publisher ship_waypoint_pub;

	  std::map<int, Tug::Point> current_waypoints_; //int is for tug_id

	  Tug::Route_around_ship route_around_ship_;
	  std::vector<int> tugs_under_my_control_;

	  double scale_;
 };

}