// tests.cpp
#include "geometry/tug_environment.hpp"
#include "tug_shortest_path.hpp"
#include "tug_route_around_ship.hpp"
#include <gtest/gtest.h>
#include <math.h>


//SHORTEST_PATH start
TEST(ShortestPathTest, NoSafetyMargin)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
 
  Tug::Polyline shortest_path_test;

  Tug::Point start(30, 180, tug_environment);
  Tug::Point finish(190, 87, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  VisiLibity::Polyline shortest_path_solution;
  shortest_path_solution.push_back(VisiLibity::Point(30,180));
  shortest_path_solution.push_back(VisiLibity::Point(120,190));
  shortest_path_solution.push_back(VisiLibity::Point(170,170));
  shortest_path_solution.push_back(VisiLibity::Point(190,87));

  ASSERT_EQ(shortest_path_test.size(), shortest_path_solution.size());

  for (int i = 0; i < shortest_path_solution.size(); ++i)
  {
    ASSERT_EQ(shortest_path_test[i].x(), shortest_path_solution[i].x());
    ASSERT_EQ(shortest_path_test[i].y(), shortest_path_solution[i].y());
  }
}

TEST(ShortestPathTest, StartAndFinishWithinSafetyMargin)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(44);

  Tug::Polyline shortest_path_test;
  Tug::Point start(30, 180, tug_environment);
  Tug::Point finish(347, 230, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  Tug::Polyline shortest_path_solution;
  shortest_path_solution.push_back(Tug::Point(30,180, tug_environment));
  shortest_path_solution.push_back(Tug::Point(26,180, tug_environment));
  shortest_path_solution.push_back(Tug::Point(26,200, tug_environment));
  shortest_path_solution.push_back(Tug::Point(120, 237, tug_environment));
  shortest_path_solution.push_back(Tug::Point(176, 216, tug_environment));
  shortest_path_solution.push_back(Tug::Point(347, 216, tug_environment));
  shortest_path_solution.push_back(Tug::Point(347, 230, tug_environment));

  ASSERT_EQ(shortest_path_test.size(), shortest_path_solution.size());

  for (int i = 0; i < shortest_path_solution.size(); ++i)
  {
    ASSERT_EQ(shortest_path_test[i].x(), shortest_path_solution[i].x());
    ASSERT_EQ(shortest_path_test[i].y(), shortest_path_solution[i].y());
  }

}

TEST(ShortestPathTest, NoValidPath)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(54);

  Tug::Polyline shortest_path_test;
  Tug::Point start(30, 180, tug_environment);
  Tug::Point finish(347, 180, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  ASSERT_EQ(shortest_path_test.size(), 0);
}

TEST(ShortestPathTest, TrivialCase)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  Tug::Polyline shortest_path_test;
  Tug::Point start(30, 180, tug_environment);
  Tug::Point finish(20, 180, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  ASSERT_EQ(shortest_path_test[0].x(), start.x());
  ASSERT_EQ(shortest_path_test[0].y(), start.y());
  ASSERT_EQ(shortest_path_test[1].x(), finish.x());
  ASSERT_EQ(shortest_path_test[1].y(), finish.y());

}

TEST(ShortestPathTest, StartPointInsideObstacle)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  Tug::Polyline shortest_path_test;
  Tug::Point start(90, 150, tug_environment);
  Tug::Point finish(347, 180, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment, start, finish, shortest_path_test);

  ASSERT_EQ(shortest_path_test.size(), 0);
}

TEST(ShortestPathTest, AllPairsShortestPathNoSafetyMargin)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
 
  Tug::Polyline shortest_path_test;

  Tug::Point start(30, 180, tug_environment);
  Tug::Point finish(190, 87, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment);
  shortest_path_class.calculate_shortest_path(start, finish, shortest_path_test, tug_environment);

  Tug::Polyline shortest_path_solution;
  shortest_path_solution.push_back(Tug::Point(30,180,tug_environment));
  shortest_path_solution.push_back(Tug::Point(120,190, tug_environment));
  shortest_path_solution.push_back(Tug::Point(170,170, tug_environment));
  shortest_path_solution.push_back(Tug::Point(190,87, tug_environment));

  ASSERT_LE(shortest_path_test.length(), shortest_path_solution.length());

}

TEST(ShortestPathTest, AllPairsShortestPathNoValidPath)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(54);

  Tug::Polyline shortest_path_test;

  Tug::Point start(2, 7, tug_environment);
  Tug::Point finish(340, 3, tug_environment);

  Tug::Shortest_path shortest_path_class(tug_environment);

  bool ok = shortest_path_class.calculate_shortest_path(start, finish, shortest_path_test, tug_environment);

  ASSERT_EQ(shortest_path_test.size(), 0);
}
//SHORTEST_PATH end

TEST(RouteAroundShipTest, correctPlacement)
{

  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  Tug::Point mid_pt(10, 10, tug_environment);
  Tug::Route_around_ship route_around_ship(M_PI/2, 2, 4);

  Tug::Point start(14, 10, tug_environment);
  Tug::Point finish(7, 9, tug_environment);

  Tug::Polyline solution = route_around_ship.best_route(start, finish, tug_environment);

  std::cout << "solution: ";
  for (int i = 0; i < solution.size(); ++i)
  {
    std::cout << solution[i] << std::endl;
  }
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}