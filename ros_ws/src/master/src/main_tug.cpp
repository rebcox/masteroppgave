#include "ros/ros.h"
#include "tug_constants.hpp"
#include "tug_waypoint_publisher.hpp"

//#define SCALE 220.0

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "tug_waypoint_publisher");
    int id;
    if (argc != 2){ROS_ERROR("need tug ID as argument"); return -1;};
    try
    {
      id = *argv[1] - '0';
    }
    catch(...)
    {
      ROS_ERROR("need tug ID argument to be an integer"); return -1;
    }

    ROS_INFO("Tug node with id %d is initialized", id);
    Tug::Waypoint_publisher wp(id, SCALE);
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("pose", 20, &Tug::Waypoint_publisher::update_position, &wp);
    ros::Subscriber path_sub = node.subscribe("paths", 20, &Tug::Waypoint_publisher::set_path, &wp);

    ros::spin();
    return 0;
  }
