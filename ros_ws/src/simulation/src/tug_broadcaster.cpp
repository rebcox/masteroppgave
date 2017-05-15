#include <ros/ros.h>
#include <ros/package.h>
#include <turtlesim/Pose.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>

std::string tug_name;

/*void poseCallback(const geometry_msgs::PoseConstPtr& msg){

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->position.x+2, msg->position.y, 0.0) );
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  //q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  //transform.setRotation(msg->orientation);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", tug_name));
}*/
void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", tug_name));
}

int main(int argc, char** argv){
  //PURPOSE: BROADCAST WHERE I AM RELATIVE TO THE WORLD
  ros::init(argc, argv, "tug_broadcaster");
  if (argc != 2){ROS_ERROR("need tug name as argument"); return -1;};
  tug_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(tug_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

