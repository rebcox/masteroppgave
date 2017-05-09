#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "ros/ros.h"
#include <sstream>
#include <string>
#include "geometry/tug_environment.hpp"
#include "geometry/tug_point.hpp"
#include "external/visilibity.hpp"


int main( int argc, char** argv )
{
  ros::init(argc, argv, "simple_map");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher obstacle_pub = n.advertise<visualization_msgs::MarkerArray>("the_obstacles", 1);

  std::string package_path = ros::package::getPath("simulation");

  ROS_INFO("Loading environment file ");
  //std::string environment_file(argv[1]);
  std::string environment_file = "ex1tug.txt";
  std::stringstream environment_file_path;
  environment_file_path << package_path << "/include/simulation/" << environment_file; 

  Tug::Environment env(environment_file_path.str(), 1, 0.001);
  VisiLibity::Environment vis_env = env.visilibity_environment();


  while (ros::ok())
  {
    visualization_msgs::MarkerArray messages;

    for (int i = 0; i < vis_env.h()+1; ++i)
    {
      visualization_msgs::Marker poly;
      poly.header.frame_id = "/my_frame";
      poly.header.stamp = ros::Time::now();
      poly.type = visualization_msgs::Marker::LINE_STRIP;
      poly.action = visualization_msgs::Marker::ADD;
      poly.id = i;
      poly.color.r = 1.0f;
      poly.color.g = 0.0f;
      poly.color.b = 0.0f;
      poly.color.a = 1.0;
      poly.lifetime = ros::Duration();
      poly.pose.position.x = 0;
      poly.pose.position.y = 0;
      poly.pose.orientation.w = 1.0;
      poly.scale.x = 1.0;
      poly.ns = "obst";

      VisiLibity::Polygon p = vis_env[i];
      for (int i = 0; i < p.n(); ++i)
      {
        VisiLibity::Point point = p[i];
        geometry_msgs::Point pt_mes;
        pt_mes.x = (int)point.x();
        pt_mes.y = (int)point.y();
        pt_mes.z = 0;

        poly.points.push_back(pt_mes);
      }
      VisiLibity::Point point = p[0];
      geometry_msgs::Point pt_mes;
      pt_mes.x = (int)point.x();
      pt_mes.y = (int)point.y();
      pt_mes.z = 0;

      poly.points.push_back(pt_mes);
      messages.markers.push_back(poly);
    }

    // Publish the marker
    while (obstacle_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the obstacles");
      sleep(1);
    }

    obstacle_pub.publish(messages);

    r.sleep();
  }
}
