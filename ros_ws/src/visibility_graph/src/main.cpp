#include "clipper.hpp"  
#include "iostream"

/*void DrawPolygons(ClipperLib::Paths paths)
{
  for (int i = 0; i<paths.size(); i++)
  {
    for (int j = 0; j < paths[i].size(); j++)
    {
      paths[i][j].X
      paths[i][j].Y


    }
  }
  std::cout << "|";
  for (int i = 0; i < 10; i++)
  {
    std::cout << " ";
  }
  std::cout << "x";
    for (int i = 0; i < 10; i++)
  {
    std::cout << " ";
  }
    std::cout << "|";

}*/

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

//using namespace std;
//using namespace ClipperLib;
class SVGBuilder
{
  static std::string ColorToHtml(unsigned clr)
  {
    std::stringstream ss;
    ss << '#' << std::hex << std::setfill('0') << std::setw(6) << (clr & 0xFFFFFF);
    return ss.str();
  }
  //------------------------------------------------------------------------------

  static float GetAlphaAsFrac(unsigned clr)
  {
    return ((float)(clr >> 24) / 255);
  }
  //------------------------------------------------------------------------------

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
  static const std::string svg_xml_start[];
  static const std::string poly_end[];

public:
  StyleInfo style;

  void AddPaths(ClipperLib::Paths& poly)
  {
    if (poly.size() == 0) return;
    polyInfos.push_back(PolyInfo(poly, style));
  }

  bool SaveToFile(const std::string& filename, double scale = 1.0, int margin = 10)
  {
    //calculate the bounding rect ...
    PolyInfoList::size_type i = 0;
    ClipperLib::Paths::size_type j;
    while (i < polyInfos.size())
    {
      j = 0;
      while (j < polyInfos[i].paths.size() &&
        polyInfos[i].paths[j].size() == 0) j++;
      if (j < polyInfos[i].paths.size()) break;
      i++;
    }
    if (i == polyInfos.size()) return false;

    ClipperLib::IntRect rec;
    rec.left = polyInfos[i].paths[j][0].X;
    rec.right = rec.left;
    rec.top = polyInfos[i].paths[j][0].Y;
    rec.bottom = rec.top;
    for ( ; i < polyInfos.size(); ++i)
      for (ClipperLib::Paths::size_type j = 0; j < polyInfos[i].paths.size(); ++j)
        for (ClipperLib::Path::size_type k = 0; k < polyInfos[i].paths[j].size(); ++k)
        {
          ClipperLib::IntPoint ip = polyInfos[i].paths[j][k];
          if (ip.X < rec.left) rec.left = ip.X;
          else if (ip.X > rec.right) rec.right = ip.X;
          if (ip.Y < rec.top) rec.top = ip.Y;
          else if (ip.Y > rec.bottom) rec.bottom = ip.Y;
        }

    if (scale == 0) scale = 1.0;
    if (margin < 0) margin = 0;
    rec.left = (ClipperLib::cInt)((double)rec.left * scale);
    rec.top = (ClipperLib::cInt)((double)rec.top * scale);
    rec.right = (ClipperLib::cInt)((double)rec.right * scale);
    rec.bottom = (ClipperLib::cInt)((double)rec.bottom * scale);
    ClipperLib::cInt offsetX = -rec.left + margin;
    ClipperLib::cInt offsetY = -rec.top + margin;

    std::ofstream file;
    file.open(filename);
    if (!file.is_open()) return false;
    file.setf(std::ios::fixed);
    file.precision(0);
    file << svg_xml_start[0] <<
      ((rec.right - rec.left) + margin*2) << "px" << svg_xml_start[1] <<
      ((rec.bottom - rec.top) + margin*2) << "px" << svg_xml_start[2] <<
      ((rec.right - rec.left) + margin*2) << " " <<
      ((rec.bottom - rec.top) + margin*2) << svg_xml_start[3];
    setlocale(LC_NUMERIC, "C");
    file.precision(2);

    for (PolyInfoList::size_type i = 0; i < polyInfos.size(); ++i)
  {
      file << " <path d=\"";
    for (ClipperLib::Paths::size_type j = 0; j < polyInfos[i].paths.size(); ++j)
      {
        if (polyInfos[i].paths[j].size() < 3) continue;
        file << " M " << ((double)polyInfos[i].paths[j][0].X * scale + offsetX) <<
          " " << ((double)polyInfos[i].paths[j][0].Y * scale + offsetY);
        for (ClipperLib::Path::size_type k = 1; k < polyInfos[i].paths[j].size(); ++k)
        {
          ClipperLib::IntPoint ip = polyInfos[i].paths[j][k];
          double x = (double)ip.X * scale;
          double y = (double)ip.Y * scale;
          file << " L " << (x + offsetX) << " " << (y + offsetY);
        }
        file << " z";
    }
      file << poly_end[0] << ColorToHtml(polyInfos[i].si.brushClr) <<
    poly_end[1] << GetAlphaAsFrac(polyInfos[i].si.brushClr) <<
        poly_end[2] <<
        (polyInfos[i].si.pft == ClipperLib::pftEvenOdd ? "evenodd" : "nonzero") <<
        poly_end[3] << ColorToHtml(polyInfos[i].si.penClr) <<
    poly_end[4] << GetAlphaAsFrac(polyInfos[i].si.penClr) <<
        poly_end[5] << polyInfos[i].si.penWidth << poly_end[6];

        if (polyInfos[i].si.showCoords)
        {
      file << "<g font-family=\"Verdana\" font-size=\"11\" fill=\"black\">\n\n";
      for (ClipperLib::Paths::size_type j = 0; j < polyInfos[i].paths.size(); ++j)
      {
        if (polyInfos[i].paths[j].size() < 3) continue;
        for (ClipperLib::Path::size_type k = 0; k < polyInfos[i].paths[j].size(); ++k)
        {
          ClipperLib::IntPoint ip = polyInfos[i].paths[j][k];
          file << "<text x=\"" << (int)(ip.X * scale + offsetX) <<
          "\" y=\"" << (int)(ip.Y * scale + offsetY) << "\">" <<
          ip.X << "," << ip.Y << "</text>\n";
          file << "\n";
        }
      }
      file << "</g>\n";
        }
    }
    file << "</svg>\n";
    file.close();
    setlocale(LC_NUMERIC, "");
    return true;
  }
}; //SVGBuilder


const std::string SVGBuilder::svg_xml_start [] =
  {"<?xml version=\"1.0\" standalone=\"no\"?>\n"
    "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.0//EN\"\n"
    "\"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">\n\n"
    "<svg width=\"",
    "\" height=\"",
    "\" viewBox=\"0 0 ",
    "\" version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\">\n\n"
  };
const std::string SVGBuilder::poly_end [] =
  {"\"\n style=\"fill:",
    "; fill-opacity:",
    "; fill-rule:",
    "; stroke:",
    "; stroke-opacity:",
    "; stroke-width:",
    ";\"/>\n\n"
  };

//------------------------------------------------------------------------------
// Miscellaneous function ...
//------------------------------------------------------------------------------

bool SaveToFile(const std::string& filename, ClipperLib::Paths &ppg, double scale = 1.0, unsigned decimal_places = 0)
{
  std::ofstream ofs(filename);
  if (!ofs) return false;

  if (decimal_places > 8) decimal_places = 8;
  ofs << std::setprecision(decimal_places) << std::fixed;

  ClipperLib::Path pg;
  for (size_t i = 0; i < ppg.size(); ++i)
  {
    for (size_t j = 0; j < ppg[i].size(); ++j)
      ofs << ppg[i][j].X / scale << ", " << ppg[i][j].Y / scale << "," << std::endl;
    ofs << std::endl;
  }
  ofs.close();
  return true;
}
//------------------------------------------------------------------------------

bool LoadFromFile(ClipperLib::Paths &ppg, const std::string& filename, double scale)
{
  //file format assumes: 
  //  1. path coordinates (x,y) are comma separated (+/- spaces) and 
  //  each coordinate is on a separate line
  //  2. each path is separated by one or more blank lines

  ppg.clear();
  std::ifstream ifs(filename);
  if (!ifs) return false;
  std::string line;
  ClipperLib::Path pg;
  while (std::getline(ifs, line))
  {
    std::stringstream ss(line);
    double X = 0.0, Y = 0.0;
    if (!(ss >> X))
    {
      //ie blank lines => flag start of next polygon 
      if (pg.size() > 0) ppg.push_back(pg);
      pg.clear();
      continue;
    }
    char c = ss.peek();  
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces before comma
    if (c == ',') {ss.read(&c, 1); c = ss.peek();} //gobble comma
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces after comma
    if (!(ss >> Y)) break; //oops!
    pg.push_back(ClipperLib::IntPoint((ClipperLib::cInt)(X * scale),(ClipperLib::cInt)(Y * scale)));
  }
  if (pg.size() > 0) ppg.push_back(pg);
  ifs.close();
  return true;
}
//------------------------------------------------------------------------------

void MakeRandomPoly(int edgeCount, int width, int height, ClipperLib::Paths & poly)
{
  poly.resize(1);
  poly[0].resize(edgeCount);
  for (int i = 0; i < edgeCount; i++){
    poly[0][i].X = rand() % width;
    poly[0][i].Y = rand() % height;
  }
}
//------------------------------------------------------------------------------

bool ASCII_icompare(const char* str1, const char* str2)
{
  //case insensitive compare for ASCII chars only
  while (*str1) 
  {
    if (toupper(*str1) != toupper(*str2)) return false;
    str1++;
    str2++;
  }
  return (!*str2);
}

int main(int argc, char const *argv[])
{
  ClipperLib::Path subj;
  ClipperLib::Paths solution;
  subj << 
  ClipperLib::IntPoint(0,0) << 
  ClipperLib::IntPoint(0,100) << 
  ClipperLib::IntPoint(55,120) << 
  ClipperLib::IntPoint(100,100) <<  
  ClipperLib::IntPoint(100,0) << 
  ClipperLib::IntPoint(50,-20);

  /*ClipperLib::IntPoint(348,257) << ClipperLib::IntPoint(364,148) << ClipperLib::IntPoint(362,148) << 
    ClipperLib::IntPoint(326,241) << ClipperLib::IntPoint(295,219) << ClipperLib::IntPoint(258,88) << 
    ClipperLib::IntPoint(440,129) << ClipperLib::IntPoint(370,196) << ClipperLib::IntPoint(372,275);
*/
  ClipperLib::ClipperOffset co;
  co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
  co.Execute(solution, 10.0);

  for (int i = 0; i<solution.size(); i++)
  {
    for (int j = 0; j < solution[i].size(); ++j)
    {
      std::cout << i << ", " << j << " X: " << solution[i][j].X << std::endl;
      std::cout << i << ", " << j << " Y: " <<  solution[i][j].Y << std::endl;
      std::cout << std::endl;
    }

  }

  ClipperLib::Paths original;
  original.push_back(subj);

  SVGBuilder svg;
  svg.style.penWidth = 0.8;
  svg.style.brushClr = 0x1200009C;
  svg.style.penClr = 0xCCD3D3DA;
  svg.style.pft = ClipperLib::pftNonZero;
  svg.AddPaths(original);
  svg.style.brushClr = 0x129C0000;
  svg.style.penClr = 0xCCFFA07A;
  svg.style.pft = ClipperLib::pftEvenOdd;

  svg.AddPaths(solution);

  svg.SaveToFile("solution.svg", 1);


 // std::cout << point.X << std::endl;
  //std::cout << point.Y << std::endl;
  
    //draw solution ...
  //DrawPolygons(solution, 0x4000FF00, 0xFF009900);
//system("solution.svg"); 
  return 0;
}