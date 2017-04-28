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
    //this->shortest_path_costs_ = other

    //this->point_id_=other.id();
    return *this;
  }

  bool Point::is_visible(const Tug::Point &point) const
  {
    //assertion error if visibility_polygon_ is empty
    return point.in(visibility_polygon_, 0.01);
    //Does not work. Probably cause the points are just copies, and does not have id
    /*for (int i = 0; i < visible_polygons_.size(); ++i)
    {
      if (visible_polygons_[i] == point.id())
      {
        return true;
      }
    }
    return false;*/
  }

  void Point::create_visibility_polygon(const Environment &environment)
  {
    visibility_polygon_.clear();
    visible_vertices_.clear();

    visibility_polygon_ = VisiLibity::Visibility_Polygon(*this, environment.visilibity_environment(), 0.001);
    /*for (int i = 0; i < visibility_polygon_.n(); ++i)
    {
      int id = environment.find_id(visibility_polygon_[i]);
     //   std::cout << "id: " << id << std::endl;

      if (id > -1)
      {
        //std::cout << "id: " << id << std::endl;
        visible_vertices_.push_back(id);
      } 
    }*/

    for (auto i = environment.const_begin(); i != environment.const_end(); ++i)
    {
      if (i->second.in(visibility_polygon_, 0.001) and !i->second.is_on_outer_boundary)
      {
        visible_vertices_.push_back(i->first);
        //std::cout << *this << " pushed back " << i->first << " " << i->second << std::endl;
      }
    }
  
  }

  std::ostream& operator<<(std::ostream &out, Point const &pt)
  {
    out << "(" << pt.x() << ", " << pt.y() << ")";
    return out;
  }
}