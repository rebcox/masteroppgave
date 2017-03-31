// tests.cpp
//#include "tug_point.cpp"
#include "include/tug_environment.hpp"
//#include "tug_environment.cpp"
#include "include/shortest_path.h"

#include <gtest/gtest.h>

//TUG_POINT start
TEST(ConstructorTest, XAndYSetCorrectly) 
{ 
    Tug::Point pt(1.0,2.0);
    ASSERT_EQ(1.0, pt.x());
    ASSERT_EQ(2.0, pt.y());
}
//TUG_POINT end

//SHORTEST_PATH start
TEST(ShortestPathTest, NoSafetyMargin)
{
 // Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
 
  Tug::Polyline shortest_path_test;

  Tug::Point start(30, 180, tug_environment.visilibity_environment());
  Tug::Point finish(190, 87, tug_environment.visilibity_environment());
  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  VisiLibity::Polyline shortest_path_solution;
  shortest_path_solution.push_back(VisiLibity::Point(30,180));
  shortest_path_solution.push_back(VisiLibity::Point(120,190));
  shortest_path_solution.push_back(VisiLibity::Point(170,170));
  shortest_path_solution.push_back(VisiLibity::Point(190,87));

  for (int i = 0; i < shortest_path_solution.size(); ++i)
  {
    ASSERT_EQ(shortest_path_test[i].x(), shortest_path_solution[i].x());
    ASSERT_EQ(shortest_path_test[i].y(), shortest_path_solution[i].y());
  }
}

TEST(ShortestPathTest, StartAndFinishWithinSafetyMargin)
{
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(44);

  Tug::Polyline shortest_path_test;
  Tug::Point start(30, 180, tug_environment.visilibity_environment());
  Tug::Point finish(347, 230, tug_environment.visilibity_environment());

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  VisiLibity::Polyline shortest_path_solution;
  shortest_path_solution.push_back(VisiLibity::Point(30,180));
  shortest_path_solution.push_back(VisiLibity::Point(26,180));
  shortest_path_solution.push_back(VisiLibity::Point(26,200));
  shortest_path_solution.push_back(VisiLibity::Point(120, 237));
  shortest_path_solution.push_back(VisiLibity::Point(176, 216));
  shortest_path_solution.push_back(VisiLibity::Point(347, 216));
  shortest_path_solution.push_back(VisiLibity::Point(347, 230));

  for (int i = 0; i < shortest_path_solution.size(); ++i)
  {
    ASSERT_EQ(shortest_path_test[i].x(), shortest_path_solution[i].x());
    ASSERT_EQ(shortest_path_test[i].y(), shortest_path_solution[i].y());
  }

}
TEST(ShortestPathTest, NoValidPath)
{
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(54);

  Tug::Polyline shortest_path_test;
  Tug::Point start(30, 180, tug_environment.visilibity_environment());
  Tug::Point finish(347, 180, tug_environment.visilibity_environment());

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  ASSERT_EQ(shortest_path_test.size(), 0);
}

TEST(ShortestPathTest, TrivialCase)
{
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  Tug::Polyline shortest_path_test;
  Tug::Point start(30, 180, tug_environment.visilibity_environment());
  Tug::Point finish(20, 180, tug_environment.visilibity_environment());

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  ASSERT_EQ(shortest_path_test[0].x(), start.x());
  ASSERT_EQ(shortest_path_test[0].y(), start.y());
  ASSERT_EQ(shortest_path_test[1].x(), finish.x());
  ASSERT_EQ(shortest_path_test[1].y(), finish.y());

}

TEST(ShortestPathTest, StartPointInsideObstacle)
{
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  Tug::Polyline shortest_path_test;
  Tug::Point start(90, 150, tug_environment.visilibity_environment());
  Tug::Point finish(347, 180, tug_environment.visilibity_environment());

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  ASSERT_EQ(shortest_path_test.size(), 0);
}
//SHORTEST_PATH end

//TUG_ENVIRONMENT start
TEST(TugEnvironmentTest, PointsOnBoundaryMarkedCorrectly)
{
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(50);
  std::vector<Tug::Point> points = tug_environment.points();

  //These are manually checked that they are on boundary
  bool on_boundary[] = {1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,0,0};

  for (int i = 0; i < points.size(); ++i)
  {
    if (on_boundary[i])
    {
      EXPECT_TRUE(points[i].is_on_outer_boundary);
    }
    else
    {
      EXPECT_FALSE(points[i].is_on_outer_boundary);
    }
  }
}
TEST(TugEnvironmentTest, AllPointsWithinOuterBoundary)
{
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment_out_of_boundary.txt", 1.0, 0.01);
  Tug::Environment tug_environment("/Users/rebeccacox/GitHub/mast/oppg/environments/test_environment_out_of_boundary.txt", 1.0, 0.01);

  VisiLibity::Point pt4(350, 260); 
  VisiLibity::Point pt5(320, 260); 
  VisiLibity::Point pt6(320, 350); 
  VisiLibity::Point pt7(350, 350); 

  EXPECT_EQ(tug_environment(4).x(), pt4.x()); 
  EXPECT_EQ(tug_environment(4).y(), pt4.y());
  EXPECT_EQ(tug_environment(5).x(), pt5.x());
  EXPECT_EQ(tug_environment(5).y(), pt5.y());
  EXPECT_EQ(tug_environment(6).x(), pt6.x());
  EXPECT_EQ(tug_environment(6).y(), pt6.y());
  EXPECT_EQ(tug_environment(7).x(), pt7.x()); 
  EXPECT_EQ(tug_environment(7).y(), pt7.y()); 


}
//TUG_ENVIRONMENT end

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}