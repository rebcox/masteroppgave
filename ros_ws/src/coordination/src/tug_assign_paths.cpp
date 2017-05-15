#include "tug_assign_paths.hpp"
#include "ros/ros.h"
namespace Tug
{

  bool Assign_paths::assign_on_combined_shortest_path(std::vector<Boat> &tugs, 
                                                      const std::vector<Point> &finish_points, 
                                                      Environment &environment)
  {

    int nrows = finish_points.size();
    int ncols = tugs.size();

    if (nrows > ncols)
    {
      std::cout << "More goals than tugs" << std::endl;
      return false;
    }

    if (finish_points.size() == 1 && tugs.size()>0)
    {
      Boat *closest = &tugs[0];
      double closest_distance = euclidean_distance(tugs[0].get_position(), finish_points[0]);
      for (std::vector<Boat>::iterator tug = tugs.begin(); tug != tugs.end(); ++tug)
      {
        if (euclidean_distance(tug->get_position(), finish_points[0]) < closest_distance)
        {
          closest = &*tug;
        }
      }

      Shortest_path shortest_path_node(environment);
      Polyline sp;
      shortest_path_node.calculate_shortest_path(closest->get_position(), finish_points[0], sp, environment);
      closest->set_path(sp);
      return true;
    }

    if (nrows < ncols)
    {
      std::cout << "More tugs than goals. Choosing closest tugs" << std::endl;

      double sum_x =0; double sum_y=0;
      for (int i = 0; i < ncols; ++i)
      {
        sum_x += finish_points[i].x();
        sum_y += finish_points[i].y();
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

    Shortest_path shortest_path_node(environment);
    Polyline sp_temp;
    for (int i = 0; i < nrows; ++i)
    {
      for (int j = 0; j < ncols; ++j)
      {
        shortest_path_node.calculate_shortest_path(tugs[i].get_position(), finish_points[j], sp_temp, environment);
        cost_mat(i,j) = sp_temp.length();
      }
    }

    Munkres<double> m;
    m.solve(cost_mat);

    for ( int row = 0 ; row < nrows ; row++ ) {
      int rowcount = 0;
      for ( int col = 0 ; col < ncols ; col++  ) {
        if ( cost_mat(row,col) == 0 )
          rowcount++;
      }
      if ( rowcount != 1 )
      {
        std::cerr << "Row " << row << " has " << rowcount << " columns that have been matched." << std::endl;
        return false;        
      }
    }

    for ( int col = 0 ; col < ncols ; col++ ) {
      int colcount = 0;
      for ( int row = 0 ; row < nrows ; row++ ) {
        if ( cost_mat(row,col) == 0 )
          colcount++;
      }
      if ( colcount != 1 )
      {
        std::cerr << "Column " << col << " has " << colcount << " rows that have been matched." << std::endl;
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
          shortest_path_node.calculate_shortest_path(tugs[row].get_position(), finish_points[col], path, environment);
          tugs[row].set_path(path);
          //tugs[row].set_path(finish_points[col]);
        }
      }
      std::cout << std::endl;
    }

    std::cout << std::endl;

    return true;
  }

  bool Assign_paths::assign_on_combined_shortest_path(std::map<int, Boat> &tugs, 
                                      const std::vector<Point> &finish_points, 
                                      Environment &environment)
  {
    std::vector<Boat> temp_tugs;
    for (std::map<int, Boat>::iterator i = tugs.begin(); i != tugs.end(); ++i)
    {
      temp_tugs.push_back(i->second);
    }
    bool ok = assign_on_combined_shortest_path(temp_tugs, finish_points, environment);
    for (std::vector<Boat>::iterator i = temp_tugs.begin(); i != temp_tugs.end(); ++i)
    {
      tugs[i->id()].set_path(i->get_path());
    }
    return ok;
  }

/*
  Assign_paths::assign_on_eucleadian_length(std::vector<Boat> &tugs, 
                                            const std::vector<Point> &finish_points, 
                                            const Environment &environment)
  {
    std::vector<std::vector<double>> distances;

  }
  Assign_paths::assign_on_not_crossing(std::vector<Boat> &tugs, 
                                      const std::vector<Point> &finish_points, 
                                      const Environment &environment)
  {

  }
  */

  double Assign_paths::euclidean_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }
}

