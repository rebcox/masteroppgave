#include <opencv2/opencv.hpp>
#include <sstream>
#include <fstream>
#include <iostream>


struct PointsAndImage
{
  std::vector<std::vector<cv::Point>> obstacles;
  cv::Mat image;
  bool obstacle_submitted = true;
};

bool is_listed_clockwise(const std::vector<cv::Point> &points)
{
  //Shoelace formula. Returns true if points are listed clockwise in a regular cartesian coordinate system
  int n_points = points.size();
  int sum = 0;

  if (n_points>0)
  {
    sum += (points[0].x - points.back().x)*(points[0].y - points.back().y);
    for (int i = 0; i < n_points-1; ++i)
    {
      sum += (points[i+1].x - points[i].x) * (points[i+1].y + points[i].y);
    }
  }
  if (sum > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool drawObstacle(std::vector<cv::Point> &obstacle, cv::Mat &image)
{
  if (obstacle.size() < 3)
  {
    std::cout << "Error: Obstacle is a point or a line" << std::endl;
    return false;
  } 
  
  cv::Scalar color(0,0,255,255);

  line(image, obstacle.back(), obstacle[0], color);

  for (int i = 0; i < obstacle.size()-1; ++i)
  {
    line(image, obstacle[i], obstacle[i+1], color);
  }

  imshow("Map", image);
  return true;
}

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
  PointsAndImage *points_and_image = static_cast<PointsAndImage*>(userdata);

  if  ( event == cv::EVENT_LBUTTONDOWN )
  {
    std::cout << "Point added at position (" << x << ", " << y << ")" << std::endl;
    cv::Point pt(x,y);

    if (points_and_image->obstacle_submitted)
    {
      std::vector<cv::Point> points;
      points.push_back(pt);
      points_and_image->obstacles.push_back(points);
      points_and_image->obstacle_submitted = false;
    }
    else
    {
      points_and_image->obstacles.back().push_back(pt);
    }
    
    cv::Scalar color(0,0,255,255);
    cv::circle(points_and_image->image, pt, 2, color);
    imshow("Map", points_and_image->image);
  }

  else if (event == cv::EVENT_MBUTTONDOWN )
  {
    drawObstacle(points_and_image->obstacles.back(), points_and_image->image);
    points_and_image->obstacle_submitted = true;
    std::cout << "Obstacle added to environment" << std::endl;    
  }
}

void write_environment_to_file(const std::vector<std::vector<cv::Point>> &obstacles, const std::string &filename)
{
  std::ofstream file_output;
  file_output.open (filename);
  for (int i = 0; i < obstacles.size(); i++)
  {
    for (int j = 0; j < obstacles[i].size(); ++j)
    {
      file_output << obstacles[i][j].x << "," << obstacles[i][j].y << "\n";
    }
    file_output << "\n";
  }

  file_output.close();
}
void make_outer_boundary(const cv::Point &bottom_left, const cv::Point &upper_right, PointsAndImage &points_and_image)
{
  //Screen coordinates
  int x_min = bottom_left.x; 
  int x_max = upper_right.x; 
  int y_max = bottom_left.y; 
  int y_min = upper_right.y; 

  std::vector<cv::Point> points;
  //Outer boundary must be listet counter clockwise, that is clockwise in screen coordinates
  points.push_back(cv::Point(x_min,y_min));
  points.push_back(cv::Point(x_max,y_min));
  points.push_back(cv::Point(x_max,y_max));
  points.push_back(cv::Point(x_min,y_max));

  points_and_image.obstacles.clear();
  points_and_image.obstacles.push_back(points);

}

int main(int argc, char** argv)
{
  cv::Mat img = cv::imread("/home/sondre/demo_env.png");

  if ( img.empty() ) 
  { 
      std::cout << "Image file not found" << std::endl;
      return -1; 
  }

 // float hw = (float)img.cols/img.rows;
  float hw = (float)img.rows/img.cols;

  //cv::resize(img, img, cv::Size(900, round(900*hw)));
  cv::namedWindow("Map", cv::WINDOW_NORMAL);
  //cv::resizeWindow("Map", img.cols/2, img.rows/2); //600, round((1754/2479)*600));
  cv::imshow("Map", img);

/*
resize(image, image, Size(image.cols/2, image.rows/2)); // to half size or even smaller
namedWindow( "Display frame",CV_WINDOW_AUTOSIZE);
imshow("Display frame", image);
*/


  PointsAndImage points_and_image;
  points_and_image.image = img;

  cv::setMouseCallback("Map", callBackFunc, &points_and_image);
  
  std::cout << "Choose bottom left and top right corner of outer boundary" << std::endl;

  while(1)
  {
    char key = (char)cv::waitKey();
    if(key==27) break;
    else if (key==13)
    {
      if (points_and_image.obstacles.size()==1)
      {
        make_outer_boundary(points_and_image.obstacles[0][0], points_and_image.obstacles[0][1], points_and_image);
      }//All obstacles must be listed clockwise
      else if(points_and_image.obstacles.size()>1 && !is_listed_clockwise(points_and_image.obstacles.back()))
      {
        std::reverse(points_and_image.obstacles.back().begin(), points_and_image.obstacles.back().end());
      }

      drawObstacle(points_and_image.obstacles.back(), points_and_image.image);
      points_and_image.obstacle_submitted = true;
      std::cout << "Obstacle added to environment" << std::endl;
    }
  }             

  write_environment_to_file(points_and_image.obstacles, "environment.txt");
  std::cout << "Environment saved to file" << std::endl;

  return 0;
}
