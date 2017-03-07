#ifndef SVG_BUILDER_HPP
#define SVG_BUILDER_HPP

#include "clipper.hpp"  
#include "visilibity.hpp"

#include <cmath>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

class SVGBuilder
{
  std::string ColorToHtml(unsigned clr);
  //------------------------------------------------------------------------------

  float GetAlphaAsFrac(unsigned clr);

  class StyleInfo
  {
    public:
    ClipperLib::PolyFillType pft;
    unsigned brushClr;
    unsigned penClr;
    double penWidth;
    bool showCoords;

    StyleInfo()
    {
      pft = ClipperLib::pftNonZero;
      brushClr = 0xFFFFFFCC;
      penClr = 0xFF000000;
      penWidth = 0.8;
      showCoords = false;
    }
  };

  class PolyInfo
  {
    public:
      ClipperLib::Paths paths;
      StyleInfo si;

      PolyInfo(ClipperLib::Paths paths, StyleInfo style)
      {
          this->paths = paths;
          this->si = style;
      }
  };

  typedef std::vector<PolyInfo> PolyInfoList;

private:
  PolyInfoList polyInfos;
  VisiLibity::Polyline polyline;


public:
  StyleInfo style;

  void AddPaths(ClipperLib::Paths& poly);
  void AddPolyline(const VisiLibity::Polyline& polyline);

  bool SaveToFile(const std::string& filename, double scale, int margin);
}; //SVGBuilder

#endif