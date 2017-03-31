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
    { vertices_.clear(); polyline_edited_since_last_length_calculation = true;}

    void push_back(const Point& point_temp)
    { vertices_.push_back(point_temp); polyline_edited_since_last_length_calculation = true;}

    void pop_back()
    { vertices_.pop_back(); polyline_edited_since_last_length_calculation = true;}

    /*void set_vertices(const std::vector<Point>& vertices_temp)
    { vertices_ = vertices_temp; }*/

    void reverse(){std::reverse( std::begin(vertices_) , std::end(vertices_ ) );}

    double length()
    {
      if (polyline_edited_since_last_length_calculation)
      {
        if (vertices_.size()==0 || vertices_.size()==1)
        {
          return 0;
        }
        double sum = 0;
        
        for (int i = 0; i < vertices_.size()-1; ++i)
        {
          sum += sqrt(pow(vertices_[i].x() - vertices_[i+1].x(), 2) + pow(vertices_[i].y() - vertices_[i+1].y(), 2));
        }
        length_ = sum;
        polyline_edited_since_last_length_calculation = false;
        return sum;
      }
      else
      {
        //It has been calculated before
        return length_;
      }
    };

    bool operator==(const Polyline &p2) const
    {
      if (this->size() != p2.size())
      {
        return false;
      }
      for (int i = 0; i < this->size(); ++i)
      {
        if (this->vertices_[i] != p2[i])
        {
          return false;
        }
      }
      return true;
    }

  private:
    std::vector<Point> vertices_;
    double length_;
    bool polyline_edited_since_last_length_calculation = true;
  };

}

#endif //TUG_POLYLINE_H