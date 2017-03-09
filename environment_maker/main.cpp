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
  //Shoelace formula
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

void write_environment_to_file(const std::vector<std::vector<cv::Point>> &obstacles)
{
  std::ofstream file_output;
  file_output.open ("environment.txt");
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

int main(int argc, char** argv)
{
  cv::Mat img = cv::imread("map.jpg");

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

  while(1)
  {
    char key = (char)cv::waitKey();
    if(key==27) break;
    else if (key==13)
    {
      //First obstacle (outer boundary) must be listet counter clockwise
      if (points_and_image.obstacles.size()==1)
      {
        if (is_listed_clockwise(points_and_image.obstacles.back()))
        {
          std::reverse(points_and_image.obstacles.back().begin(), points_and_image.obstacles.back().end());
        }
      }//All other obstacles must be listed clockwise
      else if(!is_listed_clockwise(points_and_image.obstacles.back()))
      {
        std::reverse(points_and_image.obstacles.back().begin(), points_and_image.obstacles.back().end());
      }

      drawObstacle(points_and_image.obstacles.back(), points_and_image.image);
      points_and_image.obstacle_submitted = true;
      std::cout << "Obstacle added to environment" << std::endl;
    }
  }             

  write_environment_to_file(points_and_image.obstacles);
  std::cout << "yeess" << std::endl;

  return 0;
}
