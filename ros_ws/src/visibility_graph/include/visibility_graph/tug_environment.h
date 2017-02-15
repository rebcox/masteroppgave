
#ifndef TUG_ENVIRONMENT_H
#define TUG_ENVIRONMENT_H

#include "visilibity.hpp"
#include "clipper.hpp"
#include "SVG_builder.hpp"
#include <vector>

namespace Tug
{
  class Environment //: public Visilibity::Environment
  {
    public:
      Environment(const std::string& filename, double scale);
      void add_constant_safety_margin(double margin); //, ClipperLib::Paths &solution);
      bool load_from_file(ClipperLib::Paths &ppg, const std::string& filename, double scale);
      VisiLibity::Polyline shortest_path(VisiLibity::Point start, VisiLibity::Point finish, double epsilon);
      void save_environment_as_svg(const std::string filename);
      //std::vector<VisiLibity::Polygon> obstacles_with_safety_margin_;
    private:
      VisiLibity::Environment visilibity_environment_;
      std::vector<VisiLibity::Polygon> holes_;
      VisiLibity::Polygon outer_boundary_;
      ClipperLib::Paths paths_;
      ClipperLib::Paths paths_with_safety_margin_;
      void offset_polygon(VisiLibity::Polygon &polygon, int margin);
      void convert_to_visilibity_environment(const ClipperLib::Paths &paths);
      void path_to_hole(const ClipperLib::Path &path, VisiLibity::Polygon &hole);
      void create_visibility_graph(double epsilon);
  };
}

#endif //TUG_ENVIRONMENT_H

