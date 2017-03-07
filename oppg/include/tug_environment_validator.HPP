
#ifndef TUG_ENVIRONMENT_VALIDATOR_H
#define TUG_ENVIRONMENT_VALIDATOR_H

#include "clipper.hpp"

namespace Tug
{
  class Environment_validator
  {
  public:
    bool is_valid_environment(ClipperLib::Paths &paths);
    bool path_intersect(const ClipperLib::Path &path1, const ClipperLib::Path &path2);
    bool is_within_outer_boundary(const ClipperLib::Path &path);
  };
}
#endif //TUG_ENVIRONMENT_VALIDATOR_H