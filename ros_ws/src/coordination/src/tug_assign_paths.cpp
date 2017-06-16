/*
 *   Copyright (c) 2007 John Weaver
 *   Copyright (c) 2015 Miroslav Krajicek
 *   Copyright (c) 2017 Rebecca Cox
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include "tug_assign_paths.hpp"
#include "ros/ros.h"
namespace Tug
{
//OBS: This function removes tugs from vector if they are not used.
  bool Assign_paths::assign(std::vector<Boat> &tugs, 
                            const std::vector<Point> &goal_pts, 
                            Environment &environment)
  {
    int nrows = goal_pts.size();
    int ncols = tugs.size();

    if (nrows > ncols) {std::cout << "More goals than tugs" << std::endl;return false;}

    if (goal_pts.size() == 1 && tugs.size()>0)
    {
      Boat *closest = &tugs[0];
      double closest_distance = euclidean_distance(tugs[0].get_position(), 
      goal_pts[0]);
      for (std::vector<Boat>::iterator tug = tugs.begin(); tug != tugs.end(); ++tug)
      {
        if (euclidean_distance(tug->get_position(), goal_pts[0]) < closest_distance)
        {
          closest = &*tug;
        }
      }

      Polyline sp;
      shortest_path_node_ptr->calculate_shortest_path(closest->get_position(), 
                                                      goal_pts[0], 
                                                      sp,
                                                      environment);
      closest->set_path(sp);
      return true;
    }

    if (nrows < ncols)
    {
      std::cout << "More tugs than goals. Choosing closest tugs" << std::endl;

      double sum_x =0; double sum_y=0;
      for (int i = 0; i < ncols; ++i)
      {
        sum_x += goal_pts[i].x();
        sum_y += goal_pts[i].y();
      }
      Point mid_point(sum_x/ncols, sum_y/ncols);

      double distances[nrows];

      for (int i = 0; i < nrows; ++i)
      {
        distances[i] = euclidean_distance(tugs[i].get_position(), mid_point);
      }

      int n_tugs_to_remove = ncols-nrows;
      std::vector<int> removable;

      for(int i = 0; i < n_tugs_to_remove; i++)
      {
        double *max_dist = std::max_element(distances, distances+nrows);
        int max_index = std::distance(distances, max_dist);
        distances[max_index] = 0;
        removable.push_back(max_index);
      }
      for (int i = removable.size()-1; i>=0; --i)
      {
        tugs.erase(tugs.begin()+removable[i]);
      }
      ncols = nrows;
    }

    Matrix<double> cost_mat(nrows, ncols);

    Polyline sp_temp;
    for (int i = 0; i < nrows; ++i)
    {
      for (int j = 0; j < ncols; ++j)
      {
        shortest_path_node_ptr->calculate_shortest_path(tugs[i].get_position(), 
                                                        goal_pts[j], 
                                                        sp_temp, 
                                                        environment);
        cost_mat(i,j) = sp_temp.length();
      }
    }

    Munkres<double> m;
    m.solve(cost_mat);

    for ( int row = 0 ; row < nrows ; row++ ) 
    {
      int rowcount = 0;
      for ( int col = 0 ; col < ncols ; col++ ) 
      {
        if ( cost_mat(row,col) == 0 )
        rowcount++;
      }
      if ( rowcount != 1 )
      {
        std::cerr << "Row " << row << " has " << rowcount <<
        " columns that have been matched." << std::endl;
        return false;        
      }
    }

    for ( int col = 0 ; col < ncols ; col++ ) 
    {
      int colcount = 0;
      for ( int row = 0 ; row < nrows ; row++ ) 
      {
        if ( cost_mat(row,col) == 0 )
        {
          colcount++;
        }
      }

      if ( colcount != 1 )
      {
        std::cerr << "Column " << col << " has " << colcount << 
        " rows that have been matched." << std::endl;
        return false;        
      }
    }
      // Display solved matrix.
    for ( int row = 0 ; row < nrows ; row++ ) 
    {
      bool tug_planned = false;
      for ( int col = 0 ; col < ncols ; col++ ) 
      {
        std::cout.width(2);
        std::cout << cost_mat(row,col) << ",";
        if (cost_mat(row,col) == 0)
        {
          Polyline path;
          shortest_path_node_ptr->calculate_shortest_path(tugs[row].get_position(), 
                                                          goal_pts[col], 
                                                          path,
                                                          environment);
          tugs[row].set_path(path);
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;

    return true;
  }

  bool Assign_paths::assign_on_combined_shortest_path(std::map<int, Boat> &tugs, 
                                                      const std::vector<Point> &goal_pts, 
                                                      Environment &environment)
  {
    std::vector<Boat> temp_tugs;
    for (std::map<int, Boat>::iterator i = tugs.begin(); i != tugs.end(); ++i)
    {
      temp_tugs.push_back(i->second);
    }
    bool ok = assign(temp_tugs, goal_pts, environment);

    for (std::vector<Boat>::iterator i = temp_tugs.begin(); i != temp_tugs.end(); ++i)
    {
     tugs[i->id()].set_path(i->get_path());
    }

    return ok;
  }

  bool Assign_paths::assign_on_combined_shortest_path(std::vector<Boat> &tugs, 
                                                      std::map<int, Point> &goal_pts, 
                                                      Environment &environment)
  {
    std::vector<Point> end_points;
    for (std::map<int,Point>::iterator i = goal_pts.begin(); i != goal_pts.end(); ++i)
    {
      end_points.push_back(i->second);
    }
    assign(tugs, end_points, environment);
  }

  double Assign_paths::euclidean_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }
}
