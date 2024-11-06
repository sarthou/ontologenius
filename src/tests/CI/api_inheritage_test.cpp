#include <algorithm>
#include <cstddef>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(api_inheritage, getName_call)
{
  std::string res = onto_ptr->classes.getName("human");
  EXPECT_EQ(res, "human");
}

TEST(api_inheritage, find_call)
{
  std::vector<std::string> res = onto_ptr->classes.find("human");
  EXPECT_EQ(res.size(), 1);
  EXPECT_EQ(res.front(), "human");
}

TEST(api_inheritage, getUp_call)
{
  std::vector<std::string> res = onto_ptr->classes.getUp("human");
  EXPECT_EQ(res.size(), 8);
  bool res_bool = ((std::find(res.begin(), res.end(), "entity") != res.end()) &&
                   (std::find(res.begin(), res.end(), "animate") != res.end()) &&
                   (std::find(res.begin(), res.end(), "activity") != res.end()) &&
                   (std::find(res.begin(), res.end(), "attribute") != res.end()) &&
                   (std::find(res.begin(), res.end(), "human") != res.end()) &&
                   (std::find(res.begin(), res.end(), "agent") != res.end()) &&
                   (std::find(res.begin(), res.end(), "vitality") != res.end()) &&
                   (std::find(res.begin(), res.end(), "living") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(api_inheritage, getDown_call)
{
  std::vector<std::string> res = onto_ptr->classes.getDown("human");
  EXPECT_EQ(res.size(), 4);
  bool res_bool = ((std::find(res.begin(), res.end(), "human") != res.end()) &&
                   (std::find(res.begin(), res.end(), "woman") != res.end()) &&
                   (std::find(res.begin(), res.end(), "man") != res.end()) &&
                   (std::find(res.begin(), res.end(), "child") != res.end()));

  EXPECT_TRUE(res_bool);
}

TEST(api_inheritage, getDisjoint_call)
{
  std::vector<std::string> res = onto_ptr->classes.getDisjoint("woman");
  EXPECT_EQ(res.size(), 63);
  bool res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);
}

TEST(api_inheritage, depth_call)
{
  std::vector<std::string> res = onto_ptr->classes.getUp("human", 1);
  EXPECT_EQ(res.size(), 3);
  bool res_bool = ((std::find(res.begin(), res.end(), "human") != res.end()) &&
                   (std::find(res.begin(), res.end(), "agent") != res.end()) &&
                   (std::find(res.begin(), res.end(), "living") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(api_inheritage, select_true_call)
{
  EXPECT_TRUE(onto_ptr->classes.isA("human", "entity"));
}

TEST(api_inheritage, select_false_call)
{
  EXPECT_FALSE(onto_ptr->classes.isA("human", "Animal"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_api_inheritage_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
