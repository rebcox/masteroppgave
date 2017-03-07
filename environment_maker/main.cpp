#include <opencv2/opencv.hpp>
#include <sstream>
#include <fstream>
#include <iostream>

struct ButtonClick
{
  int x = -1;
  int y = -1;
};
struct PointsAndImage
{
  std::vector<ButtonClick> points;
  cv::Mat image;
};

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{

 // std::vector<ButtonClick> *points_pointer = static_cast<std::vector<ButtonClick>*>(userdata);
  ButtonClick bc;
  PointsAndImage *points_and_image = static_cast<PointsAndImage*>(userdata);

  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    //bc.x = x; bc.y = y;
    //points_pointer->push_back(bc);
  
    bc.x = x; bc.y = y;
    points_and_image->points.push_back(bc);
    
    cv::Point pt(x,y);
    cv::Scalar color(0,0,255,255);
    cv::circle(points_and_image->image, pt, 2, color);
    imshow("Map", points_and_image->image);
  }
  else if (event == cv::EVENT_MBUTTONDOWN )
  {
    std::cout << "Middle button of the mouse is clicked. Obstacle added to environment" << std::endl;

    points_and_image->points.push_back(bc);    
  }

}

void write_environment_to_file(const std::vector<ButtonClick> &points)
{
  std::ofstream file_output;
  file_output.open ("environment.txt");
  for (int i = 0; i < points.size(); i++)
  {
    if (points[i].x == -1 && points[i].y == -1)
    {
      file_output << "\n";
    }
    else
    {
      file_output << points[i].x << "," << points[i].y << "\n";
    }
  }

  file_output.close();
}

int main(int argc, char** argv)
{
  // Read image from file 
  cv::Mat img = cv::imread("map.jpg");
  //std::cout << img.size();

  if ( img.empty() ) 
  { 
      std::cout << "Image file not found" << std::endl;
      return -1; 
  }

  cv::namedWindow("Map", 1);
  cv::imshow("Map", img);

  PointsAndImage points_and_image;
  points_and_image.image = img;

  cv::setMouseCallback("Map", callBackFunc, &points_and_image);

  cv::waitKey(0);

  write_environment_to_file(points_and_image.points);
  return 0;
}
