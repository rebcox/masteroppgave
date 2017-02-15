#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/package.h"

#include "visilibity.hpp" 
#include <sstream>
#include <string>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "visibility_graph_node");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::String>("theMessage", 1000);


  std_msgs::String msg;

  std::stringstream ss;
  ss << "The message is: "; // << argv[1];
  msg.data = ss.str();



  if(argc < 2)
  {
    ROS_ERROR("Too few input arguments. Add environment and guard files");
    exit(1);
  }

//Check input validity
  if(argc > 3){
    ROS_ERROR("Too many input arguments");
    exit(1);
  }

  std::string package_path = ros::package::getPath("visibility_graph");


  //Seed the rand() fnc w/Unix time
  //(only necessary once at the beginning of the program)
  std::srand( std::time( NULL ) ); rand();


  //Set geometric robustness constant
  //:WARNING: 
  //may need to modify epsilon for Environments with greatly varying
  //scale of features
  double epsilon = 0.000000001;
  ROS_INFO("The robustness constant epsilon is set to %f", epsilon);

  /*----------Load Geometry from Files----------*/


  //Load geometric environment model from file
  ROS_INFO("Loading environment file ");
  std::string environment_file(argv[1]);

  std::stringstream environment_file_path;
  environment_file_path << package_path << "/include/visibility_graph/" << environment_file; 

  //Print environment filename to screen
  ROS_INFO("%s . . .", environment_file_path.str().c_str());

  //Construct Environment object from file
  VisiLibity::Environment my_environment(environment_file_path.str());
  ROS_INFO("OK");


  //Load guard positions from file
  ROS_INFO("Loading guards file ");
  std::string guards_file(argv[2]);


  std::stringstream guards_file_path;
  guards_file_path << package_path << "/include/visibility_graph/" << guards_file; 


  //Print guards filename to screen
  ROS_INFO("%s . . .", guards_file_path.str().c_str());
  //Construct Guards object from file
  VisiLibity::Guards my_guards(guards_file_path.str());
  ROS_INFO("OK");

  /*---------Check Validity of Geometry---------*/


  //Check Environment is epsilon-valid
  ROS_INFO("Validating environment model . . . ");
  if(  my_environment.is_valid( epsilon )  )
  {
    ROS_INFO("OK");
  }
  else
  {
    ROS_WARN("Warning:  Environment model is invalid. A valid environment model must have");
    ROS_WARN("1) outer boundary and holes pairwise epsilon -disjoint simple polygons (no two features should come within epsilon of each other), ");
    ROS_WARN("2) outer boundary is oriented ccw, and"); 
    ROS_WARN("3) holes are oriented cw.");
    exit(1);
  }


  //Check Guards are all in the Environment
  ROS_INFO("Checking all guards are in the environment and noncolocated . . . ");
  my_guards.snap_to_boundary_of(my_environment, epsilon);
  my_guards.snap_to_vertices_of(my_environment, epsilon);
  for(unsigned i=0; i<my_guards.N(); i++)
  {
    if( !my_guards[i].in(my_environment, epsilon) )
    {
      ROS_WARN("Warning: guard not in the environment");
      exit(1);
    }
  }
  if( !my_guards.noncolocated(epsilon) )
  {
    ROS_WARN("Warning:  Some guards are colocated." );
    exit(1);
  }
  else
  {
    ROS_INFO("OK");
  }

  /*----------Print Data and Statistics to Screen----------*/


  //Environment data
  ROS_INFO("The environment model is:");
  std::stringstream environment_data;
  environment_data << my_environment;
  ROS_INFO("%s", environment_data.str().c_str());

  //Environment stats
  ROS_INFO("This environment has ");
  ROS_INFO("%d vertices, ", my_environment.n());
  ROS_INFO("%d reflex vertices, ", my_environment.r());
  ROS_INFO("%d holes, ", my_environment.h());
  ROS_INFO("area %f, ", my_environment.area());
  ROS_INFO("boundary length %f, ", my_environment.boundary_length());
  ROS_INFO("diameter %f.", my_environment.diameter());



  ROS_INFO("The guards' positions are:");
  std::stringstream guards_position;
  guards_position << my_guards;
  ROS_INFO("%s", guards_position.str().c_str());
  ROS_INFO("There are %d guards", my_guards.N());


  /*----------Compute the Visibility Polygon 
                   of a Guard Chosen by User----------*/
 /*
 
  //Prompt user
  int guard_choice(0);
  std::cout << "Which guard would you like "
      <<"to compute the visibility polygon of "
      << "(0, 1, 2, ...)? " << std::endl;
  std::cin >> guard_choice; std::cout << normal;

   
  //Compute and display visibility polygon
  VisiLibity::Visibility_Polygon
    my_visibility_polygon(my_guards[guard_choice], my_environment, epsilon);
  std::cout << "The visibility polygon is" << std::endl
      << magenta << my_visibility_polygon << normal
      << std::endl;
*/
  /*
  //To save the visibility polygon in an Environment file
  VisiLibity::Environment(my_visibility_polygon)
    .write_to_file("./example_visibility_polygon.cin", IOS_PRECISION);
  */


  //Test shartest path from 1,1 -> 2,1
  //Construct the start Point
  /*VisiLibity::Point start(1.0, 1.0);
  //Construct the finish Point
  VisiLibity::Point finish(2.0, 1.0);
*/
  VisiLibity::Point start(3.0, 0.5);
  //Construct the finish Point
  VisiLibity::Point finish(7.5, 2.0);



  VisiLibity::Polyline my_shortest_path;
  my_shortest_path = my_environment.shortest_path(start, finish, epsilon);

  ROS_INFO("length of path: %f", my_shortest_path.length());
  std::stringstream shortest_path_print;
  for (int i = 0; i < my_shortest_path.size(); ++i)
  {
    shortest_path_print << my_shortest_path[i];
    if (i < my_shortest_path.size()-1)
    {
      shortest_path_print << " - ";
    }
  }
  ROS_INFO("Shortest path: %s", shortest_path_print.str().c_str());

  //ROS_INFO("%s", msg.data.c_str());
  
  pub.publish(msg);
  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  //ros::spin();
  //This does not wait for callbacks: 
  ros::spinOnce();
  // Stop the node's resources
  ros::shutdown();

  return 0;
}