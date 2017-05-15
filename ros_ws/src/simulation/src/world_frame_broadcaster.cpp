#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "world_broadcaster");
  ros::NodeHandle node;

  std::vector<std::string> tug_names;
  for (int i = 1; i < argc; ++i)
  { 
    tug_names.push_back(argv[i]);
  }

  tf::TransformBroadcaster br;
  tf::Transform transform;

 // ros::Rate rate(10.0);
 // while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    for (int i = 0; i < tug_names.size(); ++i)
    {
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", tug_names[i]));
    }

  //  rate.sleep();
  //}
    ros::spin();
  return 0;
};