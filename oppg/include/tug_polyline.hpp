#ifndef TUG_POLYLINE_H
#define TUG_POLYLINE_H

#include "visilibity.hpp"
#include "tug_point.hpp"

namespace Tug
{
  class Polyline //: public VisiLibity::Polyline
  {
  public:
    Polyline(){};
    /*Polyline(const std::vector<Point>& vertices_temp)
    { vertices_ = vertices_temp; }*/
    
    Point operator [] (unsigned i) const
    { return vertices_[i]; }

    unsigned size() const
    { return vertices_.size(); }

    Point& operator [] (unsigned i)
    { return vertices_[i]; }

    void clear()
    { vertices_.clear(); }

    void push_back(const Point& point_temp)
    { vertices_.push_back(point_temp); }

    void pop_back()
    { vertices_.pop_back(); }

    /*void set_vertices(const std::vector<Point>& vertices_temp)
    { vertices_ = vertices_temp; }*/

    void reverse(){std::reverse( std::begin(vertices_) , std::end(vertices_ ) );}

  private:
    std::vector<Point> vertices_;
  };

}

#endif //TUG_POLYLINE_H