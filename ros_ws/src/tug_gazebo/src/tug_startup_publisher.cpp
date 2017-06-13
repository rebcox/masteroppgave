#include "ros/package.h"
#include <ros/ros.h>
#include "std_msgs/UInt8.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tug_startup_publisher");
  ros::NodeHandle n;

  ros::Publisher pub_goal = n.advertise<std_msgs::UInt8>("startup", 20);

  std_msgs::UInt8 msg1;
  msg1.data = 1;
  std_msgs::UInt8 msg2;
  msg2.data = 2;

  ros::Rate loop_rate(0.5);  
  while (ros::ok())
  {
    pub_goal.publish(msg1);
    pub_goal.publish(msg2);
    loop_rate.sleep();
    ros::spinOnce();
  }


  return 0;
}