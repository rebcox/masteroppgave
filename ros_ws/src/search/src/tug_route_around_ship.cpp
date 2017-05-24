#include <tug_route_around_ship.hpp>

namespace Tug
{
	Route_around_ship::Route_around_ship(double orientation, double width, double length)
	{
    orientation_ = 0;
    position_ = Point(-1000,-1000,-1);
    ship_mat_ = Eigen::Matrix<double,2,4>(2,4);

    calculate_corners(position_, orientation, width, length, ship_mat_);
    rotate_ship(orientation, ship_mat_);

    std::cout << ship_mat_ << std::endl;
	}
  /*Route_around_ship::Route_around_ship()
  {
    Point dummy(0,0);
    Route_around_ship(dummy, 0, 0,0);

  }*/

    
  void Route_around_ship::move(const Point &mid_pt, double orientation)
  {
    if (!ship_placed_)
    {
      position_ = Point(0,0,-1);
      ship_placed_ = true;
    }

    Eigen::Translation2d transl(mid_pt.x() - position_.x(), mid_pt.y() - position_.y());
    Eigen::Affine2d af(transl);
    position_ = mid_pt;
    ship_mat_ = af*ship_mat_;

    rotate_ship(orientation, ship_mat_);
  }

  void Route_around_ship::rotate_ship(double angle, Eigen::Matrix<double,2,4> &ship_mat)
  {
    Eigen::Translation<double, 2> trans1(-position_.x(), -position_.y());
    Eigen::Rotation2D<double> rot(orientation_ - angle);
    Eigen::Translation<double, 2> trans2(position_.x(), position_.y());

    ship_mat = trans2*rot*trans1*ship_mat;
    orientation_ = angle;

  }

  void Route_around_ship::calculate_corners(const Point &mid_pt, double orientation, double width, double length, Eigen::Matrix<double,2,4> &ship_mat)
  {
    double x = mid_pt.x();
    double y = mid_pt.y();
    double half_length = length/2;
    double half_width = width/2;

    ship_mat(0, 0) = x - half_length;
    ship_mat(1, 0) = y - half_width;
    
    ship_mat(0, 1) = x - half_length;
    ship_mat(1, 1) = y + half_width;

    ship_mat(0, 2) = x + half_length;
    ship_mat(1, 2) = y + half_width;

    ship_mat(0, 3) = x + half_length;
    ship_mat(1, 3) = y - half_width;

  }
  
  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  int Route_around_ship::orientation(const Point &p, const Point &q, const Point &r)
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
  bool Route_around_ship::do_cross(const Point &p1, const Point &q1, const Point &p2, const Point &q2)
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
 
    if (o1 != o2 && o3 != o4)
    {
      return true;      
    }
    return false;
  }

  double Route_around_ship::dist(const Point &point1, const Point &point2)
  {
    return sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }

  int Route_around_ship::min_element_index(double list[4])
  {
    int current_min_index = 0;
    double current_min = list[0];
    for (int i = 1; i < 4; ++i)
    {
      if (list[i] < current_min)
      {
        current_min_index = i;
        current_min = list[i];
      }
    }
    return current_min_index;
  }

  Polyline Route_around_ship::best_route(Point start, Point finish, const Environment &env)
  {
    Polyline route;
    Point ship[5];
    for (int i = 0; i < 4; ++i)
    {
      ship[i] = Point(ship_mat_(0,i), ship_mat_(1,i), env);
    }
    ship[4] = Point(ship_mat_(0,0), ship_mat_(1,0), env);

    bool intersection[4];
    for (int i = 0; i < 4; ++i)
    {
      intersection[i] = do_cross(ship[i], ship[i+1], start, finish);
    }

    bool no_intersection = true;
    for (int i = 0; i < 4; ++i)
    {
      if (intersection[i])
      {
        no_intersection = false;
        break;
      }  
    }
    if (no_intersection)
    {
      return route;
    }
    else if(intersection[0] && intersection[2])
    {
      double alternatives[4];
      alternatives[0] = dist(start, ship[0]) + dist(ship[0], ship[3]) + dist(ship[3], finish);
      alternatives[1] = dist(start, ship[3]) + dist(ship[3], ship[0]) + dist(ship[0], finish);
      alternatives[2] = dist(start, ship[1]) + dist(ship[1], ship[2]) + dist(ship[2], finish);
      alternatives[3] = dist(start, ship[2]) + dist(ship[2], ship[1]) + dist(ship[1], finish);

      int best = min_element_index(alternatives);

      if(best == 0){
        route.push_back(ship[0]);
        route.push_back(ship[3]);
      }
      else if(best == 1){
        route.push_back(ship[3]);
        route.push_back(ship[0]);
      }
      else if(best == 2)
      {
        route.push_back(ship[1]);
        route.push_back(ship[2]);
      }
      else
      {
        route.push_back(ship[2]);
        route.push_back(ship[1]);
      }
    }
    else if(intersection[1] && intersection[3])
    {
      double alternatives[4];
      alternatives[0] = dist(start, ship[0]) + dist(ship[0], ship[1]) + dist(ship[1], finish);
      alternatives[1] = dist(start, ship[1]) + dist(ship[1], ship[0]) + dist(ship[0], finish);
      alternatives[2] = dist(start, ship[3]) + dist(ship[3], ship[2]) + dist(ship[2], finish);
      alternatives[3] = dist(start, ship[2]) + dist(ship[2], ship[3]) + dist(ship[3], finish);

      int best = min_element_index(alternatives);

      if(best == 0)
      {
        route.push_back(ship[0]);
        route.push_back(ship[1]);
      }
      else if(best == 1)
      {
        route.push_back(ship[1]);
        route.push_back(ship[0]);
      }
      else if(best == 2)
      {
        route.push_back(ship[3]);
        route.push_back(ship[2]);
      }
      else
      {
        route.push_back(ship[2]);
        route.push_back(ship[3]);
      }
    }
    else 
    {
      if (intersection[0] && intersection[1])
      {
        route.push_back(ship[1]);
      }
      else if(intersection[1] && intersection[2])
      {
        route.push_back(ship[2]);
      }
      else if(intersection[2] && intersection[3])
      {
        route.push_back(ship[3]);
      }
      else
      {
        route.push_back(ship[0]);
      }
    }
    return route;
  }
}
