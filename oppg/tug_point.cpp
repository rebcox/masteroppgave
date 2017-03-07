#include "include/tug_point.hpp"

namespace Tug
{
    Point::Point(const ClipperLib::IntPoint &point)
    {
      set_x((double)point.X);
      set_y((double)point.Y);
    }

    void Point::set_neighbor1(Point &neighbor)
    {
      neighbor1_ = &neighbor;
    }
    void Point::set_neighbor2(Point &neighbor)
    {
      neighbor2_ = &neighbor;
    }

    std::ostream& operator<<(std::ostream &out, Point const &pt)
    {
      out << "(" << pt.x() << ", " << pt.y() << ")";
      return out;
    }

}