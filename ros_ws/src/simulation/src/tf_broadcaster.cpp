#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

std::string tug_name;

void waypointCallback(const geometry_msgs::PointConstPtr& msg){
  static tf::TransformBroadcaster br;
  //msg
  tf::Transform transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.0, 1.0, 0.0));
  //transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  //tf::Quaternion q;
  //q.setRPY(0, 0, msg->theta);
  //transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "move_base", tug_name));
}


int main(int argc, char** argv){
  tug_name = "tug1";
  ros::init(argc, argv, "tug_tf_publisher");
  ros::NodeHandle n;

  //ros::Rate r(10);

  //tf::TransformBroadcaster broadcaster;


  ros::Subscriber sub = n.subscribe("waypoints", 10, &waypointCallback);


  /*while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.0, 1.0, 0.0)),
        ros::Time::now(), "move_base", "tug_transform"));
    r.sleep();
  }*/
}


