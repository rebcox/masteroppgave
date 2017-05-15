#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tug_pose");
  ros::NodeHandle node;
  if (argc != 2){ROS_ERROR("need tug name as argument"); return -1;};
  std::string tug_name = argv[1];

  ros::Publisher tug_pose_pub =
  node.advertise<geometry_msgs::Pose>(tug_name+"/pose", 10);

  tf::TransformListener listener;
  ros::Rate rate(1.0);
  

  listener.waitForTransform("world", tug_name, ros::Time(0), ros::Duration(10.0));

  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("world", tug_name, ros::Time(0), transform);
      float x = transform.getOrigin().x();
      float y = transform.getOrigin().y();
      std::cout << "Current position: (" << x << "," << y << ")" << std::endl;
      geometry_msgs::Pose tug_pose;

      tug_pose_pub.publish(tug_pose);
    } 
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    rate.sleep();
  }
  return 0;
}