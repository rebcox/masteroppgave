#include "SVG_builder.hpp"


  const std::string svg_xml_start [] =
  {"<?xml version=\"1.0\" standalone=\"no\"?>\n"
    "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.0//EN\"\n"
    "\"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">\n\n"
    "<svg width=\"",
    "\" height=\"",
    "\" viewBox=\"0 0 ",
    "\" version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\">\n\n"
  };
const std::string poly_end [] =
  {"\"\n style=\"fill:",
    "; fill-opacity:",
    "; fill-rule:",
    "; stroke:",
    "; stroke-opacity:",
    "; stroke-width:",
    ";\"/>\n\n"
  };


  std::string SVGBuilder::ColorToHtml(unsigned clr)
  {
    std::stringstream ss;
    ss << '#' << std::hex << std::setfill('0') << std::setw(6) << (clr & 0xFFFFFF);
    return ss.str();
  }

  float SVGBuilder::GetAlphaAsFrac(unsigned clr)
  {
    return ((float)(clr >> 24) / 255);
  }

  void SVGBuilder::AddPaths(ClipperLib::Paths& poly)
  {
    if (poly.size() == 0) return;
    polyInfos.push_back(PolyInfo(poly, style));
  }

  void SVGBuilder::AddPolyline(const Tug::Polyline& polylinein)
  {
    polylines.push_back(polylinein);  
  }

  bool SVGBuilder::SaveToFile(const std::string& filename, double scale, int margin)
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
    
    for (int i = 0; i < polylines.size(); i++)
    {
      if (polylines[i].size() > 0)
      {
        file << "<polyline points=\"";
        for (int j = 0; j < polylines[i].size(); ++j)
        {
          file << (polylines[i][j].x()*scale + offsetX) << "," << (polylines[i][j].y()*scale + offsetY) << " ";
        }
        file << "\"\n";
        file << "style=\"fill:none;stroke:black;stroke-width:3\" />\n\n";
      }
    }
    file << "</svg>\n";
    file.close();
    setlocale(LC_NUMERIC, "");
    return true;
  }
 //SVGBuilder


