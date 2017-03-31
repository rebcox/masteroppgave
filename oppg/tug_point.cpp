#include "include/tug_point.hpp"

namespace Tug
{
  Point::Point(const ClipperLib::IntPoint &point,  int point_id)  : point_id_(point_id)
  {
    set_x((double)point.X);
    set_y((double)point.Y);

  }

  Point::Point(double x_temp, double y_temp, const VisiLibity::Environment &environment,  int point_id)  : point_id_(point_id)
  {
    x_ = x_temp;
    y_ = y_temp;
    create_visibility_polygon(environment);
  }

  Point::Point(const ClipperLib::IntPoint &point, const VisiLibity::Environment &environment, int point_id) : point_id_(point_id)
  {
    set_x((double)point.X);
    set_y((double)point.Y);
    create_visibility_polygon(environment);
  }

  Point::Point(const Point &obj) : point_id_(obj.point_id_)
  {
    x_ = obj.x_;
    y_ = obj.y_;
    is_on_outer_boundary = obj.is_on_outer_boundary;
    visibility_polygon_ = obj.visibility_polygon_;
  }

  Point& Point::operator=(const Point &other)
  {
    if(&other == this)
      return *this;

    this->x_ = other.x();
    this->y_ = other.y();
    this->is_on_outer_boundary = other.is_on_outer_boundary;
    this->visibility_polygon_ = other.visibility_polygon();
    //this->point_id_=other.id();
    return *this;

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

  bool Point::is_available(int t)
  {
    if (t >= schedule_.size() || schedule_[t] == 0)
    {
      return true;
    }
    return false;
  }
  void Point::set_tug(int id, int t)
  {
    if (t >= schedule_.size())
    {
      int no_extra_elements = t - schedule_.size() + 1;
      std::vector<int> zeros(no_extra_elements, 0);
      schedule_.insert(schedule_.end(), zeros.begin(), zeros.end());
    }
    schedule_[t] = id;
  }



}