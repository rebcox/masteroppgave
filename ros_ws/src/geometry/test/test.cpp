// tests.cpp
#include "tug_environment.hpp"
#include <gtest/gtest.h>

//TUG_POINT start
TEST(ConstructorTest, XAndYSetCorrectly) 
{ 
    Tug::Point pt(1.0, 2.0);
    ASSERT_EQ(1.0, pt.x());
    ASSERT_EQ(2.0, pt.y());
}
//TUG_POINT end

//TUG_ENVIRONMENT start
TEST(TugEnvironmentTest, PointsOnBoundaryMarkedCorrectly)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment.txt", 1.0, 0.01);

  tug_environment.add_constant_safety_margin(50);

  //These are manually checked that they are on boundary
  bool on_boundary[] = {1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,0,0};
  int a = 0;
  //for (int i = 0; i < points.size(); ++i)
  for (std::map<int, Tug::Point>::const_iterator pt = tug_environment.const_begin(); pt != tug_environment.const_end(); ++pt)
  {
    if (on_boundary[a])
    {
      EXPECT_TRUE(pt->second.is_on_outer_boundary);
    }
    else
    {
      EXPECT_FALSE(pt->second.is_on_outer_boundary);
    }
    a++;
  }
}

TEST(TugEnvironmentTest, AllPointsWithinOuterBoundary)
{
  Tug::Environment tug_environment("/home/rebecca/GITHUB/mast/oppg/environments/test_environment_out_of_boundary.txt", 1.0, 0.01);

  Tug::Point pt0(350, 260,tug_environment); 
  Tug::Point pt1(320, 260,tug_environment); 
  Tug::Point pt2(320, 350,tug_environment); 
  Tug::Point pt3(350, 350,tug_environment); 

  EXPECT_EQ(tug_environment(1), pt0); 
  EXPECT_EQ(tug_environment(2), pt1);
  EXPECT_EQ(tug_environment(3), pt2);
  EXPECT_EQ(tug_environment(4), pt3); 
  int a =1;
  EXPECT_EQ(a,1);

}
//TUG_ENVIRONMENT end


int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}