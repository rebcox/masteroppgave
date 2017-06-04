#include "ros/ros.h"
#include "master/AvoidShipCollision.h"
#include "master/BoatPose.h"
#include "geometry/tug_polyline.hpp" 
#include "search/tug_route_around_ship.hpp"

#ifndef SCALE
#define SCALE 40.0
#endif

namespace
{
  Tug::Route_around_ship route_around_ship_(0, 0.5*SCALE, 0.6*SCALE);
  master::BoatPose newest_pose_msg_;
}

void callback_ship_pose(const master::BoatPose::ConstPtr &msg)
{
	newest_pose_msg_ = *msg;
}

bool find_path(master::AvoidShipCollision::Request &req,
          		 master::AvoidShipCollision::Response &res)
{
	double speed = req.from.v;
	Tug::Point pt(newest_pose_msg_.x, newest_pose_msg_.y, -1);
	route_around_ship_.move(pt, newest_pose_msg_.o);

	Tug::Point start(req.from.x, req.from.y, -1);
	Tug::Point end(req.to.x, req.to.y, -1);

  Tug::Polyline points_to_move_around_ship = route_around_ship_.best_route(start, end);
  
  res.path.push_back(req.from);
  for (int i = 0; i < points_to_move_around_ship.size(); ++i)
  {
  	master::Waypoint wp;
  	wp.x = points_to_move_around_ship[i].x();
  	wp.y = points_to_move_around_ship[i].y();
  	wp.v = speed;
  	res.path.push_back(wp);
  }
  res.path.push_back(req.to);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "avoid_ship_collision_service");
  
  ros::NodeHandle node;	
  ros::Subscriber sub_shipPose = node.subscribe("shipPose", 1, callback_ship_pose);

  ros::ServiceServer service = node.advertiseService("avoid_ship_collision", find_path);

  ros::spin();

	return 0;
}
