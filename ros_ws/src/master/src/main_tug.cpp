  
  int main(int argc, char const *argv[])
  {
    Tug::Waypoint_publisher wp();
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("pose", 20, &Tug::Waypoint_publisher::update_position, &wp);
    ros::Subscriber path_sub = node.subscribe("paths", 20, &Tug::Waypoint_publisher::set_path, &wp);
    ros::Subscriber goal_update_sub = node.subscribe("goal_point_updater", 20, &Tug::Waypoint_publisher::callback_update_goal, &wp);
    //ros::Subscriber waypoints_sub = node.subscribe("current_waypoints", 20, &Tug::Waypoint_publisher::set_current_waypoints, &wp);
    return 0;
  }
