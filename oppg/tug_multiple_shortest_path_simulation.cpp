//#include "include/tug_point.hpp"
#include "include/shortest_path.h"
#include <memory>
#include "include/tug_environment.hpp"
#include "time.h"
#include <stdlib.h>
#include "math.h"
#include "sstream"

using namespace Tug;
  // Given three colinear points p, q, r, the function checks if
  // point q lies on line segment 'pr'
  bool onSegment(const Point &p, const Point &q, const Point &r)
  {
      if (q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
          q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y()))
         return true;
   
      return false;
  }
   
  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  int orientation( Point &p, Point &q, Point &r)
  {
      // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
      // for details of below formula.
      int val = (q.y() - p.y()) * (r.x() - q.x()) -
                (q.x() - p.x()) * (r.y() - q.y());
   
      if (val == 0) return 0;  // colinear
   
      return (val > 0)? 1: 2; // clock or counterclock wise
  }

  // The main function that returns true if line segment 'p1q1'
  // and 'p2q2' intersect.
  bool do_cross( Point &p1,  Point &q1,  Point &p2,  Point &q2)
  {
    //Intersectiong endpoints will never give crossing lines, exept when they are all the same
    if (p1 == p2 || p1 == q2 || q1 == p2 || q1 == q2)
    {
      return false;
    }
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
    {
      return true;      
    }
 /*
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;*/
 
    return false; // Doesn't fall in any of the above cases
  }

Point generate_random_point_within_boundaries(int x_min, int x_max, int y_min, int y_max, Environment tug_environment)
{
  int x = rand() % (x_max-x_min-1) + x_min + 1;
  int y = rand() % (y_max-y_min-1) + y_min + 1;

  return Point((double)x,(double)y, tug_environment.visilibity_environment());
}

double distance(Point point1, Point point2)
{
  return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
}

int main()
{
  srand (time(NULL));

  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/big_test.txt", 1.0, 0.01);

  int x_min, x_max, y_min, y_max;
  tug_environment.get_boundaries(x_min, x_max, y_min, y_max);

  for (int i = 0; i <= 2000; ++i)
  {
    //std::cout << "Iteration " << i << std::endl;
    Point s1 = generate_random_point_within_boundaries( x_min,  x_max,  y_min,  y_max, tug_environment);
    Point f1 = generate_random_point_within_boundaries( x_min,  x_max,  y_min,  y_max, tug_environment);
    
    Point s2 = generate_random_point_within_boundaries( x_min,  x_max,  y_min,  y_max, tug_environment);
    Point f2 = generate_random_point_within_boundaries( x_min,  x_max,  y_min,  y_max, tug_environment);

    Point s3 = generate_random_point_within_boundaries( x_min,  x_max,  y_min,  y_max, tug_environment);
    Point f3 = generate_random_point_within_boundaries( x_min,  x_max,  y_min,  y_max, tug_environment);

    double distances[6];

    Polyline sp11 = tug_environment.shortest_path(s1,f1);
    double sp11_length = sp11.length();
    Polyline sp12 = tug_environment.shortest_path(s1,f2);
    double sp12_length = sp12.length();
    Polyline sp13 = tug_environment.shortest_path(s1,f3);
    double sp13_length = sp13.length();
    Polyline sp21 = tug_environment.shortest_path(s2,f1);
    double sp21_length = sp21.length();
    Polyline sp22 = tug_environment.shortest_path(s2,f2);
    double sp22_length = sp22.length();
    Polyline sp23 = tug_environment.shortest_path(s2,f3);
    double sp23_length = sp23.length();
    Polyline sp31 = tug_environment.shortest_path(s3,f1);
    double sp31_length = sp31.length();
    Polyline sp32 = tug_environment.shortest_path(s3,f2);
    double sp32_length = sp32.length();
    Polyline sp33 = tug_environment.shortest_path(s3,f3);
    double sp33_length = sp33.length();
/*
    distances[0] = sp11_length + sp22_length + sp33_length;
    distances[1] = sp11_length + sp23_length + sp32_length;
    distances[2] = sp12_length + sp21_length + sp33_length;
    distances[3] = sp12_length + sp23_length + sp31_length;
    distances[4] = sp13_length + sp21_length + sp32_length;
    distances[5] = sp13_length + sp22_length + sp31_length;*/

    distances[0] = distance(s1,f1) + distance(s2,f2) + distance(s3,f3);
    distances[1] = distance(s1,f1) + distance(s2,f3) + distance(s3,f2);
    distances[2] = distance(s1,f2) + distance(s2,f1) + distance(s3,f3);
    distances[3] = distance(s1,f2) + distance(s2,f3) + distance(s3,f1);
    distances[4] = distance(s1,f3) + distance(s2,f1) + distance(s3,f2);
    distances[5] = distance(s1,f3) + distance(s2,f2) + distance(s3,f1);


    Polyline sp1;
    Polyline sp2;
    Polyline sp3;

    int shortest_distance_index = 0;
    for (int j = 1; j < 6; ++j)
    {
      if (distances[j] < distances[shortest_distance_index])
      {
        shortest_distance_index = j;
      }
    }
    //std::cout << i << ": shortest_distance_index: " << shortest_distance_index <<std::endl;
    switch (shortest_distance_index)
    {
      case 0:
        sp1 = sp11;
        sp2 = sp22;
        sp3 = sp33;
        break;
      case 1:
        sp1 = sp11;
        sp2 = sp23;
        sp3 = sp32;
        break;
      case 2:
        sp1 = sp12;
        sp2 = sp21;
        sp3 = sp33;
        break;
      case 3:
        sp1 = sp12;
        sp2 = sp23;
        sp3 = sp31;
        break;
      case 4:
        sp1 = sp13;
        sp2 = sp21;
        sp3 = sp32;
        break;
      case 5:
        sp1 = sp13;
        sp2 = sp22;
        sp3 = sp31;
        break;
    }

    if (sp1.size() < 3 || sp2.size() < 3 || sp3.size() < 3)
    {
      continue;
    }
    for (int k = 0; k < sp1.size()-1; ++k)
    {
      for (int j = 0; j < sp2.size()-1; ++j)
      {
        //sp1 and sp2
        if (do_cross(sp1[k], sp1[k+1], sp2[j], sp2[j+1]))
        {
          std::stringstream ss;
          ss << "env_cross12_" << k << "_" << j << ".svg";      
          std::vector<Polyline> sps;
          sps.push_back(sp1);
          sps.push_back(sp2);
          sps.push_back(sp3);          
          tug_environment.save_environment_as_svg(ss.str(), sps);     
        }
      }

      for (int l = 0; l < sp3.size()-1; ++l)
      {
        //sp1 and sp3
        if (do_cross(sp1[k], sp1[k+1], sp3[l], sp3[l+1]))
        {
          std::stringstream ss;
          ss << "env_cross13_" << k << "_" << l <<".svg";           
          std::vector<Polyline> sps;
          sps.push_back(sp1);
          sps.push_back(sp2);
          sps.push_back(sp3);          
          tug_environment.save_environment_as_svg(ss.str(), sps);
        }
      }
    }

    for (int k = 0; k < sp2.size()-1; ++k)
    {
      for (int j = 0; j < sp3.size()-1; ++j)
      {
        //sp2 and sp3
        if (do_cross(sp2[k], sp2[k+1], sp3[j], sp3[j+1]))
        {
          std::stringstream ss;
          ss << "env_cross23_" << k << "_" << j << ".svg"; 
          std::vector<Polyline> sps;
          sps.push_back(sp1);
          sps.push_back(sp2);
          sps.push_back(sp3);          
          tug_environment.save_environment_as_svg(ss.str(), sps);
        }
      }
    }

  }
  return 0;
}
