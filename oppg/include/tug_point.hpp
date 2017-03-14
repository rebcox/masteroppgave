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
    Point(const ClipperLib::IntPoint &point, int point_id=-1);
    Point(int point_id = -1) : VisiLibity::Point(), point_id_(point_id){};
    Point(double x_temp, double y_temp, const VisiLibity::Environment &environment, int point_id=-1);
    Point(const ClipperLib::IntPoint &point, const VisiLibity::Environment &environment, int point_id=-1);
    Point(double x_temp, double y_temp, int point_id=-1) : VisiLibity::Point(x_temp, y_temp), point_id_(point_id){};
    Point(VisiLibity::Point &point, int point_id=-1) : VisiLibity::Point(point.x(),point.y()), point_id_(point_id) {};
    Point(const Point &obj);

    Point &operator=(const Point &other);

    int id() const {return point_id_;}

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
    //void set_point_id(int point_id){point_id_ = point_id;}

    //Point *neighbor1_;
   // Point *neighbor2_;
    const int point_id_;
    VisiLibity::Visibility_Polygon visibility_polygon_;
  };
}

#endif //TUG_POINT_H