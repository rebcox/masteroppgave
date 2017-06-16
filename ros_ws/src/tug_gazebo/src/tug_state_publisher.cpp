#include "tugboat_control/Waypoint.h"

#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <ros/ros.h>
#include <sstream>

ros::Publisher state_pub;
geometry_msgs::Pose tug_pose;
ros::ServiceClient pose_srv;
std::string tug_name;
int tug_id;
geometry_msgs::Pose current_goal;

double get_distance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return sqrt(pow((p2.position.x-p1.position.x),2) + pow((p2.position.y-p1.position.y),2));
}

void update_tug_pose()
{
  gazebo_msgs::GetModelState getmodelstate;

  getmodelstate.request.model_name = tug_name;
  pose_srv.call(getmodelstate);

  tug_pose.position.x = getmodelstate.response.pose.position.x;
  tug_pose.position.y = getmodelstate.response.pose.position.y;
}

void stop_moving()
{
  geometry_msgs::Twist vel_msg;
  gazebo_msgs::ModelState state_msg;
  state_msg.model_name = tug_name;
  state_msg.reference_frame = "world";

  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  update_tug_pose();
  state_msg.twist = vel_msg;
  state_msg.pose = tug_pose;
  state_pub.publish(state_msg);
}

void callback_waypoint(const tugboat_control::Waypoint::ConstPtr &msg)
{
  if (msg->ID == tug_id)
  {
    geometry_msgs::Twist vel_msg;
    gazebo_msgs::ModelState state_msg;
    state_msg.model_name = tug_name;
    state_msg.reference_frame = "world";
    double speed = msg->v;
    double distance_tolerance = 0.5;
      
    current_goal.position.x = msg->x;
    current_goal.position.y = msg->y;  

    update_tug_pose();
      
    ros::Rate loop_rate(4);
    
    while(get_distance(tug_pose, current_goal) > distance_tolerance)
    {
      double speed_x = (current_goal.position.x - tug_pose.position.x);
      double speed_y = (current_goal.position.y - tug_pose.position.y);
      double norm_const = sqrt(pow(speed_x,2)+pow(speed_y,2));

      vel_msg.linear.x = speed*speed_x/norm_const;
      vel_msg.linear.y = speed*speed_y/norm_const;
      vel_msg.linear.z = 0;

      state_msg.twist = vel_msg;
      state_msg.pose = tug_pose;

      state_pub.publish(state_msg);
      ros::spinOnce(); //check if current_goal is updated
      loop_rate.sleep();
      ros::spinOnce(); //check if current_goal is updated
      update_tug_pose();
    } 
    
    stop_moving();
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tug_state_publisher");

  ros::NodeHandle node;
  if (argc != 3){ROS_ERROR("need tug name and ID as argument"); return -1;};
  tug_name = argv[1];

  try
  {
    tug_id = *argv[2] - '0';
  }
  catch(...)
  {
    ROS_ERROR("need tug ID argument to be an integer"); return -1;
  }

  //current_goal.position.x=0; current_goal.position.y=0;  

	srand(time(0));

  pose_srv = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
 // update_tug_pose();
 // sleep(5);
  ros::Subscriber sub_goal = node.subscribe<tugboat_control::Waypoint>("waypoint", 20, callback_waypoint);

  state_pub = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);





  ros::spin();
 
  return 0;
};