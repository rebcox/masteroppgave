namespace Tug
{
  bool Environment_validator::is_valid_environment(ClipperLib::Paths &paths)
  {
    for (int i = 1; i < paths.size()-1; ++i)
    {
      if (!is_within_outer_boundary(paths[i]))
      {
        return false;
      }
      for (int j = i+1; j < paths.size(); ++j)
      {
        if(path_intersect(paths[i], paths[j]))
        {
          std::cout << "Polygon intersection between " << i << " and " << j << "." << std::endl;
          return false;
        }
      }
    }
    return true;
  }
  bool Environment_validator::is_within_outer_boundary(const ClipperLib::Path &path)
  {
    for (int i = 0; i < path.size(); ++i)
    {
      if (path[i].X > x_max || path[i].X < x_min || path[i].Y < y_min || path[i].Y > y_max) 
      {
        return false;
      }
    }
    return true;
  }

  bool Environment_validator::path_intersect(const ClipperLib::Path &path1, const ClipperLib::Path &path2)
  {
      for (int i = 0; i < path1.size(); ++i)
      {
        if (ClipperLib::PointInPolygon(path1[i], path2) > 0)
        {
          return true;
        }
      }
      for (int i = 0; i < path2.size(); ++i)
      {
        if (ClipperLib::PointInPolygon(path2[i], path1) > 0)
        {
          return true;
        }
      }
    return false;
  }
}
