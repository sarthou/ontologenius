#include <algorithm>
#include <cstddef>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(requests_tests, getName_call)
{
  std::string res = onto_ptr->classes.getName("human");
  EXPECT_EQ(res, "human");
}

TEST(requests_tests, find_call)
{
  std::vector<std::string> res = onto_ptr->classes.find("human");
  EXPECT_EQ(res.size(), 1);
  EXPECT_EQ(res.front(), "human");
}

TEST(requests_tests, getUp_call)
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

TEST(requests_tests, getDown_call)
{
  std::vector<std::string> res = onto_ptr->classes.getDown("human");
  EXPECT_EQ(res.size(), 4);
  bool res_bool = ((std::find(res.begin(), res.end(), "human") != res.end()) &&
                   (std::find(res.begin(), res.end(), "woman") != res.end()) &&
                   (std::find(res.begin(), res.end(), "man") != res.end()) &&
                   (std::find(res.begin(), res.end(), "child") != res.end()));

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, getDisjoint_call)
{
  std::vector<std::string> res = onto_ptr->classes.getDisjoint("woman");
  EXPECT_EQ(res.size(), 63);
  bool res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, depth_call)
{
  std::vector<std::string> res = onto_ptr->classes.getUp("human", 1);
  EXPECT_EQ(res.size(), 3);
  bool res_bool = ((std::find(res.begin(), res.end(), "human") != res.end()) &&
                   (std::find(res.begin(), res.end(), "agent") != res.end()) &&
                   (std::find(res.begin(), res.end(), "living") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, select_true_call)
{
  EXPECT_TRUE(onto_ptr->classes.isA("human", "entity"));
}

TEST(requests_tests, reasoners_list_call)
{
  std::vector<std::string> res = onto_ptr->reasoners.list();
  EXPECT_GE(res.size(), 7);
  bool res_bool = ((std::find(res.begin(), res.end(), "ontologenius::ReasonerChain") != res.end()) &&
                   (std::find(res.begin(), res.end(), "ontologenius::ReasonerDictionary") != res.end()) &&
                   (std::find(res.begin(), res.end(), "ontologenius::ReasonerInverseOf") != res.end()) &&
                   (std::find(res.begin(), res.end(), "ontologenius::ReasonerNone") != res.end()) &&
                   (std::find(res.begin(), res.end(), "ontologenius::ReasonerSymmetric") != res.end()) &&
                   (std::find(res.begin(), res.end(), "ontologenius::ReasonerGeneralize") != res.end()) &&
                   (std::find(res.begin(), res.end(), "ontologenius::ReasonerRangeDomain") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, reasoner_description_call)
{
  std::string res;
  bool res_bool = true;

  res = onto_ptr->reasoners.getDescription("ontologenius::ReasonerChain");
  res_bool = res_bool && (res == "This reasoner resolve the properties chains axioms.\n - post reasoning");
  res = onto_ptr->reasoners.getDescription("ontologenius::ReasonerDictionary");
  res_bool = res_bool && (res == "This reasoner creates several alternative dictionaries to avoid too many restrictive labels.\n - post reasoning");
  res = onto_ptr->reasoners.getDescription("ontologenius::ReasonerInverseOf");
  res_bool = res_bool && (res == "This reasoner creates the inverse properties for each individual.\n - post reasoning");
  res = onto_ptr->reasoners.getDescription("ontologenius::ReasonerNone");
  res_bool = res_bool && (res == "This is an reasoner model to show how to create your own reasoner plugin\n - post reasoning\n - pre reasoning\n - periodic reasoning");
  res = onto_ptr->reasoners.getDescription("ontologenius::ReasonerSymmetric");
  res_bool = res_bool && (res == "This reasoner creates the symetric properties for each individual.\n - post reasoning");

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, select_false_call)
{
  EXPECT_FALSE(onto_ptr->classes.isA("human", "animal"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_requests_tester");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
