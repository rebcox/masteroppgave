// tests.cpp
#include "geometry/tug_boat.hpp"
#include "geometry/tug_environment.hpp"
#include "tug_assign_paths.hpp"
#include "tug_scheduler.hpp"
#include <gtest/gtest.h>

TEST(TugSchedulerTest, correctPrioritation)
{

  Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);
  Tug::Shortest_path shortest_path_class(tug_env);

  Tug::Point s1(60, 60,tug_env);
  Tug::Point f1(320,320,tug_env);
  Tug::Polyline sp1;
  shortest_path_class.calculate_shortest_path(s1, f1, sp1, tug_env);

  Tug::Point s2(40, 40,tug_env);
  Tug::Point f2(40,240,tug_env);
  Tug::Polyline sp2;
  shortest_path_class.calculate_shortest_path(s2, f2, sp2, tug_env);


  Tug::Point s3(200, 200,tug_env);
  Tug::Point f3(349, 1,tug_env);
  Tug::Polyline sp3;
  shortest_path_class.calculate_shortest_path(s3, f3, sp3, tug_env);


  Tug::Point s4(10, 10,tug_env);
  Tug::Point f4(20, 20,tug_env);
  Tug::Polyline sp4;
  shortest_path_class.calculate_shortest_path(s4, f4, sp4, tug_env);


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
  Tug::Environment tug_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);

  std::vector<Tug::Point> finish_points;
  finish_points.push_back(Tug::Point(140,205,tug_env));
  finish_points.push_back(Tug::Point(330,10,tug_env));
  finish_points.push_back(Tug::Point(315,320,tug_env));

  std::vector<Tug::Point> start_points;
  start_points.push_back(Tug::Point(105,45,tug_env)); //fp3
  start_points.push_back(Tug::Point(270,45,tug_env)); //fp2
  start_points.push_back(Tug::Point(18,25,tug_env)); //fp1

  std::map<int, Tug::Boat> tugs;

  for (int i = 0; i < start_points.size(); ++i)
  {
    Tug::Boat tug(7.0, start_points[i], &tug_env); 
    tug.set_id(i+1);
    tugs.insert(std::make_pair(i+1, tug));
  }

  Tug::Assign_paths assigner;
  assigner.assign_on_combined_shortest_path(tugs, finish_points, tug_env);

  Tug::Shortest_path shortest_path_class(tug_env);

  Tug::Polyline path1;
  Tug::Polyline path2;
  Tug::Polyline path3;
  shortest_path_class.calculate_shortest_path(tugs[0].get_position(), finish_points[2], path1,tug_env);
  shortest_path_class.calculate_shortest_path(tugs[1].get_position(), finish_points[1], path2,tug_env);
  shortest_path_class.calculate_shortest_path(tugs[2].get_position(), finish_points[0], path3,tug_env);

  EXPECT_EQ(tugs[0].get_path(), path1);
  EXPECT_EQ(tugs[1].get_path(), path2);
  EXPECT_EQ(tugs[2].get_path(), path3);
}

TEST(AllPairsShortestPath, correctMatrix)
{
  int solution[8][8] = 
    { {0, 2, 2, 4, 8, 6, 7, 8},  
      {1, 0, 3, 1, 6, 6, 7, 8},  
      {2, 2, 0, 4, 6, 6, 2, 2},  
      {1, 1, 3, 0, 8, 1, 1, 1},  
      {8, 6, 6, 1, 0, 6, 6, 8},  
      {1, 2, 3, 1, 5, 0, 7, 5},  
      {1, 2, 2, 1, 6, 6, 0, 8},  
      {1, 2, 2, 1, 5, 5, 7, 0} };

  Tug::Environment apsp_env("/home/rebecca/GITHUB/mast/oppg/environments/apsp_env.txt", 1.0, 0.01);

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

  Tug::Environment apsp_env("/home/rebecca/GITHUB/mast/oppg/environments/ex1tug.txt", 1.0, 0.01);

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