#include "include/tug_assign_paths.hpp"

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

    for (int i = 0; i < nrows; ++i)
    {
      for (int j = 0; j < ncols; ++j)
      {
        cost_mat(i,j) = environment.shortest_path(tugs[i].get_position(), finish_points[j]).length();
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
          tugs[row].set_path(finish_points[col]);
        }
      }
      std::cout << std::endl;
    }

    std::cout << std::endl;

    return true;
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

  void Assign_paths::assign_goal(Boat &tug, Point &goal)
  {
    tug.set_path(goal);
  }

  double Assign_paths::euclidean_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }
}

