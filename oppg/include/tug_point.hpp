#ifndef TUG_POINT_H
#define TUG_POINT_H

#include "math.h"
#include "clipper.hpp"
#include "visilibity.hpp"

namespace Tug
{
  class Point : public VisiLibity::Point
  {
  public:
    Point(const ClipperLib::IntPoint &point);
    Point() : VisiLibity::Point() {};
    Point(double x_temp, double y_temp) : VisiLibity::Point(x_temp, y_temp) {};
    Point(VisiLibity::Point &point) : VisiLibity::Point(point.x(),point.y()) {};
    friend std::ostream& operator<<(std::ostream &out, Point const& pt);

    void set_neighbor1(Point &neighbor);
    void set_neighbor2(Point &neighbor);
    Point *get_neighbor1(){return neighbor1_;};
    Point *get_neighbor2(){return neighbor2_;};
    
    bool is_on_outer_boundary = false;
    /*void set_in_on_outer_boundary(bool b) {is_on_outer_boundary = b;};
    bool get_is_on_outer_boundary(){return is_on_outer_boundary;};*/
  private:
    Point *neighbor1_;
    Point *neighbor2_;
  };
}


 /* ClipperLib::IntPoint tug_to_clipperlib(tPoint point)
  {
    return ClipperLib::IntPoint(round(point.x()), round(point.y()));
  }

  VisiLibity::Point tug_to_visilibity(tPoint point)
  {
    return VisiLibity::Point(point.x(), point.y());
  }

  ClipperLib::IntPoint visilibity_to_clipperlib(VisiLibity::Point point)
  {
    return ClipperLib::IntPoint(point.x(), point.y());
  }

  VisiLibity::Point clipperlib_to_visilibity(ClipperLib::IntPoint point)
  {
    return VisiLibity::Point(point.X, point.Y);
  }*/
#endif //TUG_POINT_H