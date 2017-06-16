#include "ros/ros.h"
#include "tugboat_control/AvoidShipCollision.h"
#include "tugboat_control/BoatPose.h"
#include "geometry/tug_polyline.hpp" 
#include "search/tug_route_around_ship.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "tug_constants.hpp"
//#ifndef SCALE
//#define SCALE 220.0
//#endif

namespace
{
  Tug::Route_around_ship route_around_ship_(0, 0.6*SCALE, 0.7*SCALE);
  tugboat_control::BoatPose newest_pose_msg_;
  ros::Publisher tug_corner_pub;
}

void callback_ship_pose(const tugboat_control::BoatPose::ConstPtr &msg)
{
  tugboat_control::BoatPose new_bp;
  new_bp.x = msg->x*SCALE;
  new_bp.y = msg->y*SCALE;
  new_bp.o = msg->o;
  new_bp.ID = msg->ID;
	newest_pose_msg_ = new_bp;
  //2 next lines: Move ship so it is drawn on screen. Not necessary for service to work
  Tug::Point pt(newest_pose_msg_.x, newest_pose_msg_.y, -1);
  route_around_ship_.move(pt, newest_pose_msg_.o);

  std::vector<double> points = route_around_ship_.ship_corners();
  if (points.size()>0)
  {
    std_msgs::Float64MultiArray points_msg;
    points_msg.data = points;
    tug_corner_pub.publish(points_msg);
  }
}

bool find_path(tugboat_control::AvoidShipCollision::Request &req,
          		 tugboat_control::AvoidShipCollision::Response &res)
{
	double speed = req.from.v;
	Tug::Point pt(newest_pose_msg_.x, newest_pose_msg_.y, -1);

	route_around_ship_.move(pt, newest_pose_msg_.o);
  Tug::Point pos = route_around_ship_.get_ship_position();

	Tug::Point start(req.from.x, req.from.y, -1);
	Tug::Point end(req.to.x, req.to.y, -1);

  Tug::Polyline points_to_move_around_ship = route_around_ship_.best_route(start, end);
  res.path.push_back(req.from);
  for (int i = 0; i < points_to_move_around_ship.size(); ++i)
  {
  	tugboat_control::Waypoint wp;
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
  newest_pose_msg_ .x = -1000;
  newest_pose_msg_ .y = -1000; 
  newest_pose_msg_ .o = 0; 
  ros::Subscriber sub_shipPose = node.subscribe("shipPose", 1, callback_ship_pose);

  ros::ServiceServer service = node.advertiseService("avoidShipCollision", find_path);


  ros::NodeHandle node_;
  tug_corner_pub = node_.advertise<std_msgs::Float64MultiArray>("drawShip", 100);

  ros::spin();

	return 0;
}
