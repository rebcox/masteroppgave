#ifndef TUG_ENVIRONMENT_H
#define TUG_ENVIRONMENT_H

#include "visilibity.hpp"
#include "clipper.hpp"
#include "SVG_builder.hpp"
#include <vector>
#include "tug_point.hpp"
#include "tug_polyline.hpp"
//#include "a_star_search.h"

namespace Tug
{
  class Environment
  {
    public:
      friend class Shortest_path;
      Environment(const std::string& filename, double scale, double epsilon);
      const VisiLibity::Environment &visilibity_environment() const;
      void add_constant_safety_margin(double margin); //, ClipperLib::Paths &solution);
      bool load_from_file(ClipperLib::Paths &ppg, const std::string& filename, double scale);
      Polyline shortest_path(const Point &start, const Point &finish); //, double epsilon);
      void save_environment_as_svg(const std::string filename);
      void save_environment_as_svg(const std::string filename, const Polyline &shortest_path);
      bool has_safety_margin(){return environment_has_safety_margin;};
      unsigned n() const;
      const VisiLibity::Visibility_Graph &visibility_graph() const;
      const Point &operator () (unsigned k) const;
      const std::vector<Point>& points() const;
      void mark_point_as_on_boundary(unsigned k);

    private:
      VisiLibity::Environment visilibity_environment_;
      VisiLibity::Environment visilibity_environment_with_safety_margin_;
      bool environment_has_safety_margin = false;
      //VisiLibity::Polygon outer_boundary_;
      //ClipperLib::cInt x_min, x_max, y_min, y_max; //of outer boundary
      int x_min, x_max, y_min, y_max; //of outer boundary
      double epsilon_;

      ClipperLib::Paths paths_;
      ClipperLib::Paths paths_with_safety_margin_;

      std::vector<Point> points_in_environment_;
      std::vector<Point> points_in_environment_with_safety_margin_;

      VisiLibity::Visibility_Graph visibility_graph_;
      VisiLibity::Visibility_Graph visibility_graph_with_safety_margin_;

      void update_tug_point_list(const ClipperLib::Paths &paths, std::vector<Point> &tug_points);
      void offset_polygon(VisiLibity::Polygon &polygon, int margin);
      void convert_to_visilibity_environment(const ClipperLib::Paths &paths, VisiLibity::Environment &environment);
      void reverse_path(ClipperLib::Path &path);
      void path_to_hole(const ClipperLib::Path &path, VisiLibity::Polygon &hole);
      bool point_is_within_outer_boundary(const ClipperLib::IntPoint point);
      void find_max_and_min_in_path(const ClipperLib::Path &path, char coordinate, int &max_val, int &min_val);
      void set_outer_boundary(const ClipperLib::Path &outer_boundary, VisiLibity::Environment &environment);
      void mark_points_touching_outer_boundary();
      bool point_is_on_outer_boundary(const VisiLibity::Point &point);
  };
}

#endif //TUG_ENVIRONMENT_H

