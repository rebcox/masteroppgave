#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "move_base";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("tug_transform", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){

  std::string package_path = ros::package::getPath("simulation");
  ROS_INFO("Loading urdf file ");
  std::string urdf_file = "ex1tug.txt";
  std::stringstream urdf_file_path;
  urdf_file_path << package_path << "/include/simulation/" << urdf_file; 
  urdf::Model model;
  if (!model.initFile(urdf_file_path.str())){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");


  ros::init(argc, argv, "tug_tf_listener");
  ros::NodeHandle n;
 // KDL::Tree tree;
  //robot_state_publisher::RobotStatePublisher state_publisher(tree, model);

  //TransformListener object automatically subscribes to the transform message
  // topic over ROS and manages all transform data coming in over the wire. 
  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}