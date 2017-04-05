#include "include/tug_assign_paths.hpp"

namespace Tug
{

  bool Assign_paths::assign_on_combined_shortest_path(std::vector<Boat> &tugs, 
                                                      const std::vector<Point> &finish_points, 
                                                      Environment &environment)
  {

    int nrows = finish_points.size();
    int ncols = tugs.size();
    if (nrows != ncols)
    {
      std::cout << "Not the same number of tugs and goals" << std::endl;
      return false;
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

  double Assign_paths::eucledian_distance(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }
}

