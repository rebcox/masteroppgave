
#ifndef TUG_BOAT_H
#define TUG_BOAT_H

namespace Tug
{
  class Boat
  {
  public:
    Boat(double length, double width);
    double get_lenght(){return length_;};
    double get_width(){return width_;};
  private:
    double length_;
    double width_;
  };
}

#endif //TUG_BOAT_H