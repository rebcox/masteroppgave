#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "tug_mover");

  ros::NodeHandle node;
  if (argc != 2){ROS_ERROR("need tug name as argument"); return -1;};
  std::string tug_name = argv[1];

  /*ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);*/

  ros::Publisher tug_vel =
    node.advertise<geometry_msgs::Twist>(tug_name+"/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 0;
    vel_msg.linear.x = 5;
    tug_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};