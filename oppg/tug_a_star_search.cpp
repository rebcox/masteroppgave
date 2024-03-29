#include "include/tug_a_star_search.hpp"
#include <cmath>

namespace Tug
{
  A_star_search::A_star_search(const Point &start,
                              const Point &finish, //const VisiLibity::Visibility_Graph &visibility_graph,
                              const VisiLibity::Visibility_Graph &visibility_graph,
                              const VisiLibity::Visibility_Polygon &start_visibility_polygon,
                              const VisiLibity::Visibility_Polygon &finish_visibility_polygon,
                              const std::vector<Point> &points,
                              //const std::vector<bool> &points_to_remove,
                              VisiLibity::Polyline &shortest_path,
                              double epsilon)
  {
    epsilon_ = epsilon;
    shortest_path = best_first_search(start, 
                                      finish, 
                                      visibility_graph,
                                      start_visibility_polygon,
                                      finish_visibility_polygon,
                                      points); //,
                                      //points_to_remove);
  }

  double A_star_search::heurestic(const Point &point1, const Point &point2)
  {
    return eucledian_distance(point1, point2);
  }

  double A_star_search::eucledian_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }


  bool A_star_search::trivial_case(const Point &start,
                                  const Point &finish,
                                  const VisiLibity::Visibility_Polygon &start_visibility_polygon,
                                  VisiLibity::Polyline &shortest_path_output)
  {
    if( heurestic(start,finish) <= epsilon_ )
    {
      shortest_path_output.push_back(start);
      return true;
    }
    else if( finish.in(start_visibility_polygon, epsilon_) )
    {
      shortest_path_output.push_back(start);
      shortest_path_output.push_back(finish);
      return true;
    }
    return false;
  }

  void A_star_search::attach_child(Shortest_Path_Node *child, Shortest_Path_Node *current_node, 
                                  std::vector<Shortest_Path_Node> &children,
                                  const VisiLibity::Environment &environment, 
                                  const Point &finish,
                                  int index)
  {
    /*
      child.setBestParent(parent);
      parent.children.add(child);
      child.setG(parent.getG()+ child.getCost());
      child.setH(calculteH(child));

            child.vertex_index = index;
            child.parent_search_tree_location = current_node.search_tree_location;
            child.cost_to_come = heurestic( start , environment(index) );
            child.estimated_cost_to_go = heurestic( environment(index) , finish );     
            children.push_back( child );

*/
/*

            child.vertex_index = i;
              child.parent_search_tree_location = current_node.search_tree_location;
              
              child.cost_to_come = current_node.cost_to_come
                + heurestic( environment(current_node.vertex_index), environment(i) );

              child.estimated_cost_to_go = heurestic( environment(i) , finish );
              
              children.push_back( child );
*/

      //Shortest_Path_Node child;
     /* child->vertex_index = index;
      child->parent_search_tree_location = current_node->search_tree_location;

      child->cost_to_come = current_node->cost_to_come +
                           heurestic( points_in_environment[current_node->vertex_index], points_in_environment[index] );

      child->estimated_cost_to_go = heurestic( points_in_environment[index] , finish );*/
      //children.push_back( child );
     // return child;
  }

  VisiLibity::Polyline A_star_search::best_first_search(const Point &start,
                                                        const Point &finish,
                                                        const VisiLibity::Visibility_Graph &visibility_graph,
                                                        const VisiLibity::Visibility_Polygon &start_visibility_polygon,
                                                        const VisiLibity::Visibility_Polygon &finish_visibility_polygon,
                                                        const std::vector<Point> points_in_environment) //,
                                                        //const std::vector<bool> &points_to_remove)
  {
    
    VisiLibity::Polyline shortest_path_output;

    if(trivial_case(start, finish, start_visibility_polygon, shortest_path_output))
    {
      return shortest_path_output;
    }
    //Connect start and finish Points to the visibility graph
    bool *start_visible;  //start row of visibility graph
    bool *finish_visible; //finish row of visibility graph
    start_visible = new bool[points_in_environment.size()];
    finish_visible = new bool[points_in_environment.size()];
    
    for(unsigned k=0; k<points_in_environment.size(); k++)
    {
      if( !points_in_environment[k].is_on_outer_boundary &&
           points_in_environment[k].in( start_visibility_polygon , epsilon_ )  )
      {
        start_visible[k] = true;        
      }
      else
      {
        start_visible[k] = false;        
      }
      if( !points_in_environment[k].is_on_outer_boundary &&
           points_in_environment[k].in( finish_visibility_polygon , epsilon_ )  )
      {
        finish_visible[k] = true;        
      }
      else
      {
        finish_visible[k] = false;
      }
    }

    //Initialize search tree of visited nodes
    std::list<Shortest_Path_Node> T;
    //:WARNING:
    //If T is a vector it is crucial to make T large enough that it will not be resized.  If T were resized,
    // any iterators pointing to its contents would be invalidated, thus causing the program to fail.
    //T.reserve( points_in_environment.size() + 3 );

    //Initialize priority queue of unexpanded nodes
    std::set<Shortest_Path_Node> Q;
    //Construct initial node
    Shortest_Path_Node current_node;
    //convention vertex_index == points_in_environment.size() => corresponds to start Point
    //vertex_index == points_in_environment.size() + 1 => corresponds to finish Point
    current_node.vertex_index = points_in_environment.size();
    current_node.cost_to_come = 0;
    current_node.estimated_cost_to_go = heurestic(start, finish );
    //Put in T and on Q
    T.push_back( current_node );
    T.begin()->search_tree_location = T.begin();
    current_node.search_tree_location = T.begin();
    T.begin()->parent_search_tree_location = T.begin();
    current_node.parent_search_tree_location = T.begin();
    Q.insert( current_node );

    //Initialize temporary variables
    Shortest_Path_Node child; 
    //children of current_node
    std::vector<Shortest_Path_Node> children;
    //flags
    bool solution_found = false;
    bool child_already_visited = false;

    //-----------Begin Main Loop-----------
    while( !Q.empty() )
    {
      //Pop top element off Q onto current_node
      current_node = *Q.begin();
      Q.erase( Q.begin() );
      //Check for goal state (if current node corresponds to finish)
      if( current_node.vertex_index == points_in_environment.size() + 1 )
      {
        solution_found = true;
        break;
      }
      //Expand current_node (compute children)
      children.clear();
      //if current_node corresponds to start
      if( current_node.vertex_index == points_in_environment.size() )
      {
        //loop over environment vertices
        for(unsigned i=0; i < points_in_environment.size(); i++)
        {
          if( start_visible[i] )
          {
            //attach_child(&child, &current_node, children, environment, finish, i);
            child.vertex_index = i;
            child.parent_search_tree_location = current_node.search_tree_location;
            child.cost_to_come = heurestic( start , points_in_environment[i] );
            child.estimated_cost_to_go = heurestic( points_in_environment[i] , finish );    
            children.push_back( child );
          }
        }
      }
      //else current_node corresponds to a vertex of the environment
      else
      {
        //check which environment vertices are visible
        for(unsigned i=0; i < points_in_environment.size(); i++)
        {
          if( current_node.vertex_index != i )
          {
            if( visibility_graph( current_node.vertex_index , i ) && !points_in_environment[i].is_on_outer_boundary)//!points_to_remove[i] )
            {
              //attach_child(&child, &current_node, children, environment, finish, i);

              child.vertex_index = i;
              child.parent_search_tree_location = current_node.search_tree_location;
              
              child.cost_to_come = current_node.cost_to_come
                + heurestic( points_in_environment[current_node.vertex_index], points_in_environment[i] );

              child.estimated_cost_to_go = heurestic( points_in_environment[i] , finish );
              
              children.push_back( child );
            }
          }
        }
        //check if finish is visible
        if( finish_visible[ current_node.vertex_index ] )
        {
          //attach_child(&child, &current_node, children, environment, finish, (points_in_environment.size() + 1));
  
          child.vertex_index = points_in_environment.size() + 1; //finish point
          child.parent_search_tree_location = current_node.search_tree_location;

          child.cost_to_come = current_node.cost_to_come
            + heurestic( points_in_environment[current_node.vertex_index] , finish );
          child.estimated_cost_to_go = 0;
          children.push_back( child );
        }

      }
      //Process children
      for( std::vector<Shortest_Path_Node>::iterator children_itr = children.begin(); 
                                                                      children_itr != children.end();
                                                                        children_itr++ )
      {
        child_already_visited = false;

        //Check if child state has already been visited (by looking in search tree T) 
        for( std::list<Shortest_Path_Node>::iterator T_itr = T.begin(); T_itr != T.end(); T_itr++ )
        {
          if( children_itr->vertex_index == T_itr->vertex_index )
          {
            children_itr->search_tree_location = T_itr;
            child_already_visited = true;
            break;
          }    
        } 

        if( !child_already_visited )
        {
          //Add child to search tree T
          T.push_back( *children_itr );
          (--T.end())->search_tree_location = --T.end();
          children_itr->search_tree_location = --T.end();
          Q.insert( *children_itr );
        }
        else if( children_itr->search_tree_location->cost_to_come > children_itr->cost_to_come )
        {
          //redirect parent pointer in search tree
          children_itr->search_tree_location->parent_search_tree_location
            = children_itr->parent_search_tree_location;
          //and update cost data
          children_itr->search_tree_location->cost_to_come = children_itr->cost_to_come;
          //update Q
          for(std::set<Shortest_Path_Node>::iterator Q_itr = Q.begin();
                                                       Q_itr!= Q.end();
                                                         Q_itr++)
          {
            if( children_itr->vertex_index == Q_itr->vertex_index )
            {
              Q.erase( Q_itr );
              break;
            }   
          }
          Q.insert( *children_itr );    
        }

        if( !child_already_visited )
        {
          Q.insert( *children_itr );
        }
      }
    }
    //-----------End Main Loop-----------

    //Recover solution
    if( solution_found )
    {
      reconstruct_path(shortest_path_output, current_node, start, finish, points_in_environment);
    }
    //free memory
    delete [] start_visible;
    delete [] finish_visible;
    return shortest_path_output;
  }

  void A_star_search::reconstruct_path(VisiLibity::Polyline &shortest_path_output,
                                      Shortest_Path_Node &current_node,
                                      const Point &start,
                                      const Point &finish,
                                      const std::vector<Point> &points_in_environment)
{
      shortest_path_output.push_back( finish );
      std::list<Shortest_Path_Node>::iterator backtrace_itr = current_node.parent_search_tree_location;
      
      Point waypoint;

      while( true )
      {
        if( backtrace_itr->vertex_index < points_in_environment.size() )
        {
          waypoint = points_in_environment[ backtrace_itr->vertex_index ];          
        }
        else if( backtrace_itr->vertex_index == points_in_environment.size() )
        {
          waypoint = start;          
        }
        //Add vertex if not redundant
        if( shortest_path_output.size() > 0
            and heurestic( shortest_path_output[ shortest_path_output.size()- 1 ],
              waypoint ) > epsilon_ )
        {
          shortest_path_output.push_back( waypoint );          
        }
        if( backtrace_itr->cost_to_come == 0 )
        {
          break;          
        }
        backtrace_itr = backtrace_itr->parent_search_tree_location;
      }
      shortest_path_output.reverse(); 
  }
}



  /*PriorityQueue<Point, double> frontier;
  unordered_map<Point, double> cost_so_far;
  unordered_map<Point, Point> came_from;
  frontier.put(start, 0);
  */

/*  came_from[start];
  cost_so_far[start];

  while(!frontier.empty())
  {
    auto current = frontier.get();

    if (current == goal)
    {
      break;
    }

    for auto next : current.neighbors
    {
      double next_cost = cost_so_far[current] + eucidian_distance(current, next);

      //next visited for the first time or the new cost is better than previous cost
      if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) 
      {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, goal);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }*/