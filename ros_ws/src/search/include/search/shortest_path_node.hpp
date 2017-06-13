/**
 * \file visilibity.cpp
 * \author Karl J. Obermeyer
 * \date March 20, 2008
 * \Modified by Rebecca Cox June 13, 2017
\remarks
VisiLibity:  A Floating-Point Visibility Algorithms Library,
Copyright (C) 2008  Karl J. Obermeyer (karl.obermeyer [ at ] gmail.com)

This file is part of VisiLibity.

VisiLibity is free software: you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your
option) any later version.

VisiLibity is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
License for more details.

You should have received a copy of the GNU Lesser General Public
License along with VisiLibity.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SHORTEST_PATH_NODE_H
#define SHORTEST_PATH_NODE_H

namespace Tug
{
  class Shortest_Path_Node
    {
    public:
      //flattened index of corresponding Environment vertex
      //convention vertex_index = n() => corresponds to start Point
      //vertex_index = n() + 1 => corresponds to finish Point
      unsigned vertex_index;
      //pointer to self in search tree.
      std::list<Shortest_Path_Node>::iterator search_tree_location;
      //pointer to parent in search tree.
      std::list<Shortest_Path_Node>::iterator parent_search_tree_location;
      //Geodesic distance from start Point.
      double cost_to_come;
      //Euclidean distance to finish Point.
      double estimated_cost_to_go;
      //std::vector<Shortest_Path_Node> expand();
      bool operator < (const Shortest_Path_Node& spn2) const
      {
        double f1 = this->cost_to_come + this->estimated_cost_to_go;
        double f2 = spn2.cost_to_come + spn2.estimated_cost_to_go;
        if( f1 < f2 )
          return true;
        else if( f2 < f1 )
          return false;
        else if( this->vertex_index < spn2.vertex_index )
          return true;
        else if( this->vertex_index > spn2.vertex_index )
          return false;
        else if( &(*(this->parent_search_tree_location))
           < &(*(spn2.parent_search_tree_location)) )
          return true;
        else
          return false;
      }
      // print member data for debugging
      void print() const
      {
  std::cout << "         vertex_index = " << vertex_index << std::endl
      << "parent's vertex_index = "
      << parent_search_tree_location->vertex_index
      << std::endl
      << "         cost_to_come = " << cost_to_come << std::endl
      << " estimated_cost_to_go = "
      << estimated_cost_to_go << std::endl;
      }
    };
  }

  #endif //SHORTEST_PATH_NODE_H