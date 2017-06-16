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
  std_msgs::UInt8 msg3;
  msg3.data = 3;
    std_msgs::UInt8 msg4;
  msg4.data = 4;
    std_msgs::UInt8 msg5;
  msg5.data = 5;
    std_msgs::UInt8 msg6;
  msg6.data = 6;

  ros::Rate loop_rate(0.5);  
  while (ros::ok())
  {
    pub_goal.publish(msg1);
    pub_goal.publish(msg2);
    pub_goal.publish(msg3);
    pub_goal.publish(msg4);
    pub_goal.publish(msg5);
    pub_goal.publish(msg6);

    loop_rate.sleep();
    ros::spinOnce();
  }


  return 0;
}