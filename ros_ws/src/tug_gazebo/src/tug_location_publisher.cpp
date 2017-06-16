#include "master/tug_constants.hpp"
#include "tugboat_control/BoatPose.h"
#include "tugboat_control/Waypoint.h"

#include <gazebo_msgs/GetModelState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>

ros::ServiceClient pose_srv;
ros::Publisher pub;
std::vector<std::string> tug_names_;
std::map<int,std::string> tugs_;

void update_tug_pose()
{
  gazebo_msgs::GetModelState getmodelstate;

  for (std::map<int, std::string>::iterator tug = tugs_.begin(); 
                                            tug != tugs_.end(); 
                                            ++tug)
  {
    getmodelstate.request.model_name = tug->second.c_str();
    getmodelstate.request.relative_entity_name = "world";
    try
    {
      pose_srv.call(getmodelstate);
      tugboat_control::BoatPose pose_msg;
      pose_msg.ID = tug->first;
      pose_msg.x = getmodelstate.response.pose.position.x/SCALE;
      pose_msg.y = getmodelstate.response.pose.position.y/SCALE;
      pub.publish(pose_msg);
    }
    catch(...)
    {
      ROS_ERROR("Cannot find pose info about %s", tug->second.c_str());
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tug_location_publisher");
  int i = 1;
  while (i < argc)
  {
    std::string name = argv[i];
    int tug_id = *argv[++i] - '0';
    ROS_INFO("Tug %s with id %d, ready to publish pose", name.c_str(), tug_id);
    tugs_.insert(std::pair<int, std::string>(tug_id, name));
    i++;
  }

  ros::NodeHandle node;
  pub = node.advertise<tugboat_control::BoatPose>("pose", 20);
  pose_srv = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  ros::Rate loop_rate(12);
  while (ros::ok())
  {
    update_tug_pose();
    loop_rate.sleep();
  }
  return 0;
}
