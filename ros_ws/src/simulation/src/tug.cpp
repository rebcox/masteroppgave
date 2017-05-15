#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>


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


  ros::init(argc, argv, "tug");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}



