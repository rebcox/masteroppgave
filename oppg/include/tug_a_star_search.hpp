#ifndef TUG_A_STAR_SEARCH_H
#define TUG_A_STAR_SEARCH_H

#include "tug_environment.hpp"
#include "shortest_path_node.hpp"
#include "tug_point.hpp"
//#include "visilibity.hpp"

namespace Tug
{
  class A_star_search
  {
  public:
    A_star_search(const Point &start,
                  const Point &finish, //const VisiLibity::Visibility_Graph &visibility_graph,
                  const VisiLibity::Visibility_Graph &visibility_graph,
                  const VisiLibity::Visibility_Polygon &start_visibility_polygon,
                  const VisiLibity::Visibility_Polygon &finish_visibility_polygon,
                  const std::vector<Point> &points,
                  //const std::vector<bool> &points_to_remove,
                  VisiLibity::Polyline &shortest_path,
                  double epsilon);
    ~A_star_search(){};
    VisiLibity::Polyline best_first_search(const Point &start,
                                          const Point &finish,
                                          const VisiLibity::Visibility_Graph &visibility_graph,
                                          const VisiLibity::Visibility_Polygon &start_visibility_polygon,
                                          const VisiLibity::Visibility_Polygon &finish_visibility_polygon,
                                          const std::vector<Point> points_in_environment); //,
                                          //const std::vector<bool> &points_to_remove);
  private:
    double epsilon_;
    double heurestic(const Point &point1, const Point &point2);
    double eucledian_distance(const Point &point1, const Point &point2);
    bool   trivial_case(const Point &start,
                        const Point &finish,
                        const VisiLibity::Visibility_Polygon &start_visibility_polygon,
                        VisiLibity::Polyline &shortest_path_output);
    void attach_child(Shortest_Path_Node *child, Shortest_Path_Node *current_node, 
                      std::vector<Shortest_Path_Node> &children,
                      const VisiLibity::Environment &environment,
                      const Point &finish,
                      int index);
    void reconstruct_path(VisiLibity::Polyline &shortest_path_output,
                          Shortest_Path_Node &current_node,
                          const Point &start,
                          const Point &finish,
                          const std::vector<Point> &points_in_environment);

  };
}

#endif //TUG_A_STAR_SEARCH_H