#include "ros/package.h"
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "tugboat_control/ClearWaypoint.h"

std_msgs::UInt8MultiArray waypTugs_msg;


void callback_clearWaypoint(const tugboat_control::ClearWaypoint::ConstPtr &msg)
{
  int id = msg->tugID;
  std_msgs::UInt8MultiArray temp = waypTugs_msg;
  waypTugs_msg.data.clear();
  for (int i = 0; i < temp.data.size(); ++i)
  {
    if (temp.data[i] != id)
    {
      waypTugs_msg.data.push_back(temp.data[i]);
    }
  }
    
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypTugs");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::UInt8MultiArray>("waypTugs", 20);

  ros::Subscriber sub = n.subscribe("clearWaypoint", 20, callback_clearWaypoint);

  waypTugs_msg.data.push_back(1);
  waypTugs_msg.data.push_back(2);

  ros::Rate loop_rate(1);  
  while (ros::ok())
  {
    pub.publish(waypTugs_msg);
    loop_rate.sleep();
    ros::spinOnce();
  }


  ros::spin();
  return 0;
}