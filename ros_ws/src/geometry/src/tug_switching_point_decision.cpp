//https://books.google.no/books?id=oR3sBgAAQBAJ&pg=PA258&lpg=PA258&dq=alongtrack+distance+fossen&source=bl&ots=LVVP3c__LD&sig=xtjqSUCA19GqUMS-y2JibmtAZe4&hl=no&sa=X&ved=0ahUKEwjGy7Kvle3TAhWJjCwKHbl6AQIQ6AEIKTAA#v=onepage&q=alongtrack%20distance%20fossen&f=false
#include "tug_switching_point_decision.hpp"
#include <cmath>
#include "ros/ros.h"
namespace Tug
{
  Switching_point_decision::Switching_point_decision(Point &pt1, Point &pt2)
  {
    angle_ = angle(pt1, pt2);
    start_pt_ = &pt1;
    finish_pt_ = &pt2;

    distance_between_points_ = sqrt(pow(pt1.x() - pt2.x(), 2) + pow(pt1.y() - pt2.y(), 2));
    //distance_between_points_ = along_track_distance(pt1, pt2, angle_);

  }

  double Switching_point_decision::along_track_distance(const Point &position, const Point &finish_pt, double angle)
  {
    return  sqrt(pow(position.x() - finish_pt.x(), 2) + pow(position.y() - finish_pt.y(), 2));
    //return (position.x()-finish_pt.x())*std::cos(angle) + (position.y()-finish_pt.y())*std::sin(angle);
  }

  double Switching_point_decision::angle(Point &pt1, Point &pt2)
  {
    return std::atan2(pt2.y() - pt1.y(), pt2.x() - pt1.x());
  }

  bool Switching_point_decision::accept(const Point &position, double radius)
  {
    /*double dist = (distance_between_points_ - along_track_distance(position, finish_pt_, angle_));
    ROS_ERROR("dist %f", dist);
    return (distance_between_points_ - along_track_distance(position, finish_pt_, angle_) <= radius);*/
    double dist = ( along_track_distance(position, *finish_pt_, angle_));
    ROS_ERROR("dist %f. finish: (%f, %f)", dist, finish_pt_->x(), finish_pt_->y());
    return (along_track_distance(position, *finish_pt_, angle_) <= radius);
  }
}
