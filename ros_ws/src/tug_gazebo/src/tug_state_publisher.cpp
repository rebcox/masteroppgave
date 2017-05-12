#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include "gazebo_msgs/GetModelState.h"
#include <math.h>

ros::Publisher state_pub;
geometry_msgs::Pose tug_pose;
ros::ServiceClient pose_srv;
std::string tug_name;

double get_distance(double x1, double y1, double x2, double y2){
  return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

/*
void move(double speed, double distance, bool isForward){
   
  gazebo_msgs::ModelState state_msg;
  state_msg.model_name = tug_name;
  state_msg.reference_frame = "world";

  geometry_msgs::Twist vel_msg;
  //set a random linear velocity in the x-axis
  if (isForward)
  {
    vel_msg.linear.x =abs(speed);
  }
  else
  {
    vel_msg.linear.x =-abs(speed);
  }
  vel_msg.linear.y =0;
  vel_msg.linear.z =0;
  //set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z =0;

  double t0 = ros::Time::now().toSec();
  double current_distance = 0.0;
  ros::Rate loop_rate(10);
  do{
    state_msg.twist = vel_msg;
    state_pub.publish(state_msg);
    double t1 = ros::Time::now().toSec();
    current_distance = speed * (t1-t0);
    ros::spinOnce();
    loop_rate.sleep();
   //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
  }while(current_distance<distance);

  vel_msg.linear.x = 0;
  //state_msg.pose.position.x = distance;
  state_msg.twist = vel_msg;
  state_pub.publish(state_msg);

}*/

void update_tug_pose()
{
  gazebo_msgs::GetModelState getmodelstate;

  getmodelstate.request.model_name = tug_name;
  pose_srv.call(getmodelstate);

  tug_pose.position.x = getmodelstate.response.pose.position.x;
  tug_pose.position.y = getmodelstate.response.pose.position.y;
  tug_pose.orientation.z = getmodelstate.response.pose.orientation.z;
  //ROS_INFO("x: %f", tug_pose.position.x);
}

void move_to(geometry_msgs::Pose goal_pose, double speed, double distance_tolerance){

  geometry_msgs::Twist vel_msg;
  gazebo_msgs::ModelState state_msg;
  state_msg.model_name = tug_name;
  state_msg.reference_frame = "world";

  double speed_x = (goal_pose.position.x - tug_pose.position.x);
  double speed_y = (goal_pose.position.y - tug_pose.position.y);
  double norm_const = speed_x+speed_y;

  ros::Rate loop_rate(10);
  do{
    vel_msg.linear.x = speed*speed_x/norm_const;
    vel_msg.linear.y = speed*speed_y/norm_const;
    vel_msg.linear.z = 0;

    state_msg.twist = vel_msg;
    update_tug_pose();
    state_msg.pose = tug_pose;
    state_pub.publish(state_msg);
    //ros::spinOnce();
    loop_rate.sleep();
    update_tug_pose();

  }while(get_distance(tug_pose.position.x, tug_pose.position.y, goal_pose.position.x, goal_pose.position.y)>distance_tolerance);
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  state_msg.twist = vel_msg;
  state_pub.publish(state_msg);
  update_tug_pose();
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tug_state_publisher");

  ros::NodeHandle node;
  if (argc != 2){ROS_ERROR("need tug name as argument"); return -1;};
  tug_name = argv[1];

	srand(time(0));

  

  state_pub = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);

  pose_srv = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  update_tug_pose();

  sleep(5);
  
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 7;
  goal_pose.position.y = 7;
  move_to(goal_pose, 3, 0.1);

  ros::spin();
 
  return 0;
};