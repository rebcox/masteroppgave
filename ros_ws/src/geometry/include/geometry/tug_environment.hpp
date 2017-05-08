#ifndef TUG_ENVIRONMENT_H
#define TUG_ENVIRONMENT_H

#include "external/visilibity.hpp"
#include "external/clipper.hpp"
#include "SVG_builder.hpp"
#include <vector>
#include "tug_point.hpp"
#include "tug_polyline.hpp"
#include "map"

namespace Tug
{
  class Environment
  {
    public:
      friend class Shortest_path;
      friend class All_pairs_shortest_path;
      
      Environment(const std::string& filename, double scale, double epsilon);
      Environment(){};

      const VisiLibity::Environment &visilibity_environment() const;
      void add_constant_safety_margin(double margin); //, ClipperLib::Paths &solution);
      bool load_from_file(ClipperLib::Paths &ppg, const std::string& filename, double scale);
      //Polyline shortest_path(const Point &start, const Point &finish); //, double epsilon);

      void save_environment_as_svg(const std::string filename);
      void save_environment_as_svg(const std::string filename, const Polyline &shortest_path);
      void save_environment_as_svg(const std::string filename, const std::vector<Polyline> &shortest_paths);

      bool has_safety_margin(){return environment_has_safety_margin;};
      unsigned n() const;
      const Point &operator () (unsigned k) const;

      std::vector<Point> points();

      void get_boundaries(int &x_min, int &x_max, int &y_min, int &y_max) const;
      bool point_is_within_outer_boundary(const Tug::Point point);
      bool point_is_on_outer_boundary(const VisiLibity::Point &point) const;

      int find_id(const VisiLibity::Point &point) const;

      std::map<int, Point>::const_iterator const_begin() const;
      std::map<int, Point>::const_iterator const_end() const;

      void print_coordinates_and_id() const;
      void mark_points_within_range(float range);


    private:
      int id_counter_ = 0;
      std::map<std::pair<double,double>,int> coordinate_to_id;
      bool environment_has_safety_margin = false;
      int x_min_, x_max_, y_min_, y_max_; //of outer boundary
      double epsilon_;

      VisiLibity::Environment visilibity_environment_;
      VisiLibity::Environment visilibity_environment_with_safety_margin_;

      //obstacles
      ClipperLib::Paths paths_;
      ClipperLib::Paths paths_with_safety_margin_;

      std::map<int, Point> points_in_environment_;
      std::map<int, Point> points_in_environment_with_safety_margin_;

      std::map<int, Point>::iterator begin();
      std::map<int, Point>::iterator end();

      void clip_against_outer_boundary(ClipperLib::Paths &paths_in, ClipperLib::Paths &paths_out);
      void update_tug_point_list(const ClipperLib::Paths &paths, std::map<int, Point> &tug_points);
      void make_visibility_graphs_for_points(std::map<int, Point> &tug_points);
      void offset_polygon(VisiLibity::Polygon &polygon, int margin);
      void convert_to_visilibity_environment(const ClipperLib::Paths &paths, VisiLibity::Environment &environment);
      void reverse_path(ClipperLib::Path &path);
      void path_to_hole(const ClipperLib::Path &path, VisiLibity::Polygon &hole);
      void find_max_and_min_in_path(const ClipperLib::Path &path, char coordinate, int &max_val, int &min_val);
      void set_outer_boundary(const ClipperLib::Path &outer_boundary, VisiLibity::Environment &environment);
      void mark_points_touching_outer_boundary();
  };
}

#endif //TUG_ENVIRONMENT_H

