// tests.cpp
//#include "tug_point.cpp"
#include "include/tug_environment.hpp"
//#include "tug_environment.cpp"
#include "include/shortest_path.h"
#include "include/tug_scheduler.hpp"
#include "include/tug_boat.hpp"
#include "include/tug_assign_paths.hpp"
#include "include/tug_all_pairs_shortest_path.hpp"
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
  //Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);
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

  ASSERT_EQ(shortest_path_test.size(), shortest_path_solution.size());

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

  ASSERT_EQ(shortest_path_test.size(), shortest_path_solution.size());

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
  bool on_boundary[] = {1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,0,0};

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

  VisiLibity::Point pt0(350, 260); 
  VisiLibity::Point pt1(320, 260); 
  VisiLibity::Point pt2(320, 350); 
  VisiLibity::Point pt3(350, 350); 

  EXPECT_EQ(tug_environment(1), pt0); 
  EXPECT_EQ(tug_environment(2), pt1);
  EXPECT_EQ(tug_environment(3), pt2);
  EXPECT_EQ(tug_environment(4), pt3); 
}
//TUG_ENVIRONMENT end

TEST(TugSchedulerTest, correctPrioritation)
{

  //Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);
  Tug::Environment tug_env("/Users/rebeccacox/GitHub/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);

  Tug::Point s1(60, 60,tug_env.visilibity_environment());
  Tug::Point f1(320,320,tug_env.visilibity_environment());
  Tug::Polyline sp1 = tug_env.shortest_path(s1,f1);

  Tug::Point s2(40, 40,tug_env.visilibity_environment());
  Tug::Point f2(40,240,tug_env.visilibity_environment());
  Tug::Polyline sp2 = tug_env.shortest_path(s2,f2);

  Tug::Point s3(200, 200,tug_env.visilibity_environment());
  Tug::Point f3(349, 1,tug_env.visilibity_environment());
  Tug::Polyline sp3 = tug_env.shortest_path(s3,f3);

  Tug::Point s4(10, 10,tug_env.visilibity_environment());
  Tug::Point f4(20, 20,tug_env.visilibity_environment());
  Tug::Polyline sp4 = tug_env.shortest_path(s4,f4);

  Tug::Boat tug1(7.0, s1, &tug_env); tug1.set_path(sp1); tug1.set_id(1);
  Tug::Boat tug2(7.0, s2, &tug_env); tug2.set_path(sp2); tug2.set_id(2);
  Tug::Boat tug3(7.0, s3, &tug_env); tug3.set_path(sp3); tug3.set_id(3);
  Tug::Boat tug4(7.0, s4, &tug_env); tug4.set_path(sp4); tug4.set_id(4);

  std::vector<Tug::Polyline> shortest_paths;

  std::vector<Tug::Boat> tugs {tug1, tug2, tug3, tug4};
  Tug::Scheduler tug_scheduler(tugs, tug_env);

  EXPECT_EQ(tugs[0].get_path(), sp4);
  EXPECT_EQ(tugs[1].get_path(), sp2);
  EXPECT_EQ(tugs[2].get_path(), sp3);
  EXPECT_EQ(tugs[3].get_path(), sp1);
}

TEST(AssignPathsTest, hungarianAlgorithmTest)
{
  Tug::Environment tug_env("/Users/rebeccacox/GitHub/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);
  std::vector<Tug::Point> finish_points;
  finish_points.push_back(Tug::Point(140,205,tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(330,10,tug_env.visilibity_environment()));
  finish_points.push_back(Tug::Point(315,320,tug_env.visilibity_environment()));

  std::vector<Tug::Point> start_points;
  start_points.push_back(Tug::Point(105,45,tug_env.visilibity_environment())); //fp3
  start_points.push_back(Tug::Point(270,45,tug_env.visilibity_environment())); //fp2
  start_points.push_back(Tug::Point(18,25,tug_env.visilibity_environment())); //fp1

  std::vector<Tug::Boat> tugs;

  for (int i = 0; i < start_points.size(); ++i)
  {
    Tug::Boat tug(7.0, start_points[i], &tug_env); 
    tug.set_id(i+1);
    tugs.push_back(tug);
  }

  Tug::Assign_paths assigner;
  assigner.assign_on_combined_shortest_path(tugs, finish_points, tug_env);

  Tug::Polyline path1 = tug_env.shortest_path(tugs[0].get_position(), finish_points[2]);
  Tug::Polyline path2 = tug_env.shortest_path(tugs[1].get_position(), finish_points[1]);
  Tug::Polyline path3 = tug_env.shortest_path(tugs[2].get_position(), finish_points[0]);

  EXPECT_EQ(tugs[0].get_path(), path1);
  EXPECT_EQ(tugs[1].get_path(), path2);
  EXPECT_EQ(tugs[2].get_path(), path3);
}

TEST(AllPairsShortestPath, correctMatrix)
{
  /*int solution[8][8] = 
  { {0, 2, 2, 4, 5, 6, 8, 8}, 
    {2, 2, 0, 4, 5, 5, 8, 8},  
    {1, 1, 3, 0, 1, 1, 8, 3},  
    {1, 2, 3, 1, 0, 6, 6, 8},  
    {1, 5, 5, 1, 5, 0, 7, 5},  
    {8, 8, 8, 3, 6, 6, 0, 8},  
    {1, 2, 3, 3, 5, 5, 7, 0} };*/

/*0  2  2  4  8  6  7  8  
1  0  3  1  6  6  7  8  
2  2  0  4  6  6  2  2  
1  1  3  0  1  1  1  1  
8  6  6  8  0  6  6  8  
1  2  3  1  5  0  7  5  
1  2  2  1  6  6  0  8  
1  2  2  1  5  5  7  0  */


  int solution[8][8] = 
    { {0, 2, 2, 4, 8, 6, 7, 8},  
      {1, 0, 3, 1, 6, 6, 7, 8},  
      {2, 2, 0, 4, 6, 6, 2, 2},  
      {1, 1, 3, 0, 8, 1, 1, 1},  
      {8, 6, 6, 1, 0, 6, 6, 8},  
      {1, 2, 3, 1, 5, 0, 7, 5},  
      {1, 2, 2, 1, 6, 6, 0, 8},  
      {1, 2, 2, 1, 5, 5, 7, 0} };

  Tug::Environment apsp_env("/Users/rebeccacox/GitHub/mast/oppg/environments/apsp_env.txt", 1.0, 0.01);

  Tug::All_pairs_shortest_path apsp(apsp_env);
  std::vector<std::vector<int>> apsp_matrix = apsp.get_apsp_matrix();

  EXPECT_EQ(apsp_matrix.size(), 8);
  EXPECT_EQ(apsp_matrix.at(0).size(), 8);

  for (int i = 0; i < 8; ++i)
  {
    for (int j = 0; j < 8; ++j)
    {
      EXPECT_EQ(solution[i][j], apsp_matrix.at(i).at(j));
    }
  }

}


TEST(AllPairsShortestPath, correctMatrixBig)
{

  int solution[14][14] = 
{ {0 , 2  ,2 , 4,  10,  5,  8,  8,  9,  10,  14,  12 , 13,  14} , 
{1 , 0  ,3 , 1,  10,  7,  7,  8,  9,  10,  12,  12 , 13  ,14 } ,
{2 , 2  ,0 , 4,  10,  7,  7,  8,  9,  10,  12,  12 , 2 , 2  },
{1 , 1  ,3 , 0,  10,  5,  3,  3,  1,  1 , 14 , 1 , 1 , 1  },
{10, 10, 10, 1 , 0 , 6 , 6 , 7 , 10 , 10 , 11 , 10,  12,  11},  
{10,  7 , 7,  1,  5,  0,  7,  7,  8,  5 , 5  ,10  ,12 , 11  },
{8 , 2  ,3 , 3,  6 , 6 , 0 , 8 , 8 , 5  ,12  ,9 , 9 , 9  },
{1 , 2  ,3 , 3,  6 , 7 , 7 , 0 , 9 , 9  ,12  ,9  ,9 , 9  },
{1 , 2  ,3 , 1,  10,  5,  8,  8,  0,  10,  12,  12 , 13 , 14},  
{1 , 2  ,3 , 1,  5 , 5 , 6 , 9 , 9 , 0  ,11  ,12  ,12 , 11  },
{14, 12, 12,  1 , 5 , 5 , 8 , 9 , 12 ,10 , 0  ,12  ,12 , 14  },
{1 , 2  ,3 , 1,  10,  5,  8,  9,  9,  10,  11,  0  ,13 , 11  },
{1 , 2  ,2 , 1,  10,  5,  8,  9,  9,  12,  12,  12  ,0  ,14  },
{1 , 2  ,2 , 1,  11,  5,  8,  9,  9,  11,  11,  11 , 13 , 0  } };

  Tug::Environment apsp_env("/Users/rebeccacox/GitHub/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);

  Tug::All_pairs_shortest_path apsp(apsp_env);
  std::vector<std::vector<int>> apsp_matrix = apsp.get_apsp_matrix();

  EXPECT_EQ(apsp_matrix.size(), 14);
  EXPECT_EQ(apsp_matrix[0].size(), 14);

  for (int i = 0; i < 14; ++i)
  {
    for (int j = 0; j < 14; ++j)
    {
      EXPECT_EQ(solution[i][j], apsp_matrix.at(i).at(j));
    }
  }

}




int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}