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
    Point(double x_temp, double y_temp, const VisiLibity::Environment &environment);
    Point(const ClipperLib::IntPoint &point, const VisiLibity::Environment &environment);
    Point(double x_temp, double y_temp) : VisiLibity::Point(x_temp, y_temp) {};
    Point(VisiLibity::Point &point) : VisiLibity::Point(point.x(),point.y()) {};
    Point(const Point &obj);

    friend std::ostream& operator<<(std::ostream &out, Point const& pt);
    bool is_visible(const Tug::Point &point) const;

    void set_neighbor1(Point &neighbor);
    void set_neighbor2(Point &neighbor);
   //Point *get_neighbor1(){return neighbor1_;};
   // Point *get_neighbor2(){return neighbor2_;};
    VisiLibity::Visibility_Polygon visibility_polygon() const{return visibility_polygon_;};
    void create_visibility_polygon(const VisiLibity::Environment &environment);
    
    bool is_on_outer_boundary = false;
    /*void set_in_on_outer_boundary(bool b) {is_on_outer_boundary = b;};
    bool get_is_on_outer_boundary(){return is_on_outer_boundary;};*/
  private:
    //Point *neighbor1_;
   // Point *neighbor2_;

    VisiLibity::Visibility_Polygon visibility_polygon_;
  };
}

#endif //TUG_POINT_H