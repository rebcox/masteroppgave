#include "include/tug_point.hpp"

namespace Tug
{
  Point::Point(const ClipperLib::IntPoint &point)
  {
    set_x((double)point.X);
    set_y((double)point.Y);
  }

  Point::Point(double x_temp, double y_temp, const VisiLibity::Environment &environment)
  {
    x_ = x_temp;
    y_ = y_temp;
    create_visibility_polygon(environment);
  }

  Point::Point(const ClipperLib::IntPoint &point, const VisiLibity::Environment &environment)
  {
    set_x((double)point.X);
    set_y((double)point.Y);
    create_visibility_polygon(environment);
  }

  Point::Point(const Point &obj)
  {
    x_ = obj.x_;
    y_ = obj.y_;
    is_on_outer_boundary = obj.is_on_outer_boundary;
    visibility_polygon_ = obj.visibility_polygon_;
  }

  bool Point::is_visible(const Tug::Point &point) const
  {
    //assertion error if visibility_polygon_ is empty
    return point.in(visibility_polygon_, 0.01);
  }


  void Point::create_visibility_polygon(const VisiLibity::Environment &environment)
  {
    visibility_polygon_ = VisiLibity::Visibility_Polygon(*this, environment, 0.001);    
  }

  void Point::set_neighbor1(Point &neighbor)
  {
    //neighbor1_ = &neighbor;
  }
  void Point::set_neighbor2(Point &neighbor)
  {
    //neighbor2_ = &neighbor;
  }

  std::ostream& operator<<(std::ostream &out, Point const &pt)
  {
    out << "(" << pt.x() << ", " << pt.y() << ")";
    return out;
  }

}