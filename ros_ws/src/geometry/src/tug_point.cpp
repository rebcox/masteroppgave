#include "tug_point.hpp"
#include "tug_environment.hpp"

namespace Tug
{
  Point::Point(const ClipperLib::IntPoint &point,  int point_id)  : point_id_(point_id)
  {
    set_x((double)point.X);
    set_y((double)point.Y);

  }

  Point::Point(double x_temp, double y_temp, const Environment &environment,  int point_id)  : point_id_(point_id)
  {
    x_ = x_temp;
    y_ = y_temp;
    //visibility_polygon_ = VisiLibity::Visibility_Polygon(*this, environment.visilibity_environment(), 0.001);
    create_visibility_polygon(environment);

  }

  Point::Point(const ClipperLib::IntPoint &point, const Environment &environment, int point_id) : point_id_(point_id)
  {
    set_x((double)point.X);
    set_y((double)point.Y);
    //create_visibility_polygon(environment);
  }

  Point::Point(const Point &obj) : point_id_(obj.point_id_)
  {
    x_ = obj.x_;
    y_ = obj.y_;
    is_on_outer_boundary = obj.is_on_outer_boundary;
    visibility_polygon_ = obj.visibility_polygon_;
    visible_vertices_ = obj.visible_vertices_;
  }

  Point &Point::operator=(const Point &other)
  {
    if(&other == this)
      return *this;

    this->x_ = other.x();
    this->y_ = other.y();
    this->is_on_outer_boundary = other.is_on_outer_boundary;
    this->visibility_polygon_ = other.visibility_polygon();
    this->visible_vertices_ = other.visible_vertices();
    return *this;
  }

  bool Point::operator==(const Point &other) const 
  {
    if (this->x() == other.x() && this->y() == other.y())
    {
      return true;
    }
    else return false;
  }

  bool Point::is_visible(const Tug::Point &point) const
  {
    return point.in(visibility_polygon_, 0.01);
  }

  void Point::create_visibility_polygon(const Environment &environment)
  {
    visibility_polygon_.clear();
    visible_vertices_.clear();

    visibility_polygon_ = VisiLibity::Visibility_Polygon(*this, 
                                      environment.visilibity_environment(), 0.001);

    for (auto i = environment.const_begin(); i != environment.const_end(); ++i)
    {
      if (i->second.in(visibility_polygon_, 0.001) and !i->second.is_on_outer_boundary)
      {
        visible_vertices_.push_back(i->first);
      }
    }
  
  }

  void Point::add_close_point(Point *pt)
  {
    if (pt != this)
    {
      close_points.push_back(pt);
    }
  }


  std::ostream& operator<<(std::ostream &out, Point const &pt)
  {
    out << "(" << pt.x() << ", " << pt.y() << ")";
    return out;
  }
}