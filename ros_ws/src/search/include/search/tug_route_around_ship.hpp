#ifndef TUG_ROUTE_AROUND_SHIP
#define TUG_ROUTE_AROUND_SHIP

#include "geometry/tug_point.hpp"
#include "geometry/tug_polyline.hpp"

#include <Eigen/Dense>

namespace Tug
{
class Route_around_ship
  {
  public:
  	Route_around_ship(double orientation, double width, double length);
   // Route_around_ship(){Route_around_ship(0, 0,0);}; //{position_ = Point(0,0); orientation_ = 0; };
    void move(const Point &mid_pt, double orientation);
    Polyline best_route(Point start, Point finish, const Environment &env);
  private:
    void rotate_ship(double angle, Eigen::Matrix<double,2,4> &ship_mat_);
    void calculate_corners(const Point &mid_pt, double orientation, double width, double length, Eigen::Matrix<double,2,4> &ship_mat_);
    int orientation(const Point &p,const Point &q,const Point &r);
    bool do_cross(const Point &p1, const Point &q1, const Point &p2, const Point &q2);
    double dist(const Point &point1, const Point &point2);
    int min_element_index(double list[4]);

    Eigen::Matrix<double,2,4> ship_mat_;
    double ship_corners_[4][2];
    Point position_;
    double orientation_;
  };
}

#endif //TUG_ROUTE_AROUND_SHIP