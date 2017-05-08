#ifndef TUG_POINT_H
#define TUG_POINT_H

#include "math.h"
#include "external/clipper.hpp"
#include "external/visilibity.hpp"

#include "map"

namespace Tug
{
  class Environment;

  class Point : public VisiLibity::Point
  {
  public:
    Point(const ClipperLib::IntPoint &point, int point_id=-1);
    Point(int point_id = -1) : VisiLibity::Point(), point_id_(point_id){};
    Point(double x_temp, double y_temp, const Environment &environment, int point_id=-1);
    Point(const ClipperLib::IntPoint &point, const Environment &environment, int point_id=-1);
    Point(double x_temp, double y_temp, int point_id=-1) : VisiLibity::Point(x_temp, y_temp), point_id_(point_id){};
    Point(VisiLibity::Point &point, int point_id=-1) : VisiLibity::Point(point.x(),point.y()), point_id_(point_id) {};
    Point(const Point &obj);

    Point &operator=(const Point &other);
    bool operator==(const Point &other) const; 

    int id() const {return point_id_;}


    friend std::ostream& operator<<(std::ostream &out, Point const& pt);
    bool is_visible(const Tug::Point &point) const;
    std::vector<int> visible_vertices() const {return visible_vertices_;}
    void add_shortest_path_cost(const Point &pt, double cost)
      {shortest_path_costs_.insert(std::pair<int, double>(pt.id(),cost));}
    double get_cost_to(int id) const {return shortest_path_costs_.at(id);}

    VisiLibity::Visibility_Polygon visibility_polygon() const{return visibility_polygon_;};
    void create_visibility_polygon(const Environment &environment);
    bool is_on_outer_boundary = false;
    std::vector<int> visible_vertices_;

    void add_close_point(Point *pt);
    
  protected:
    std::map<int, double> shortest_path_costs_;
  //  std::map<int, Tug::Point> visible_vertices_;
   // std::vector<int> visible_vertices_;
    const int point_id_;
    VisiLibity::Visibility_Polygon visibility_polygon_;
    std::vector<Point*> close_points; 
  };
}

#endif //TUG_POINT_H