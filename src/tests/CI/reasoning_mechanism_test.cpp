#include <algorithm>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_mechanism, reasoners_list_call)
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

TEST(reasoning_mechanism, reasoner_description_call)
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

TEST(reasoning_mechanism, reasoners_effect)
{
  std::vector<std::string> res;

  EXPECT_TRUE(onto_ptr->actions.clear());

  EXPECT_TRUE(onto_ptr->reasoners.deactivate("ontologenius::ReasonerChain"));
  EXPECT_TRUE(onto_ptr->reasoners.deactivate("ontologenius::ReasonerDictionary"));
  EXPECT_TRUE(onto_ptr->reasoners.deactivate("ontologenius::ReasonerInverseOf"));
  EXPECT_TRUE(onto_ptr->reasoners.deactivate("ontologenius::ReasonerNone"));
  EXPECT_TRUE(onto_ptr->reasoners.deactivate("ontologenius::ReasonerSymmetric"));

  std::string path_base = ros::package::getPath("ontologenius");
  std::string path = path_base + "/files/attribute.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  path = path_base + "/files/positionProperty.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  path = path_base + "/files/test_individuals.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  // ReasonerSymmetric

  res = onto_ptr->individuals.getOn("red_cube", "isInFrontOf");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "blue_book") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerSymmetric"));

  res = onto_ptr->individuals.getOn("red_cube", "isInFrontOf");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "blue_book") != res.end());

  // ReasonerInverseOf

  res = onto_ptr->individuals.getOn("blue_cube", "isUnder");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "green_cube") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerInverseOf"));

  res = onto_ptr->individuals.getOn("blue_cube", "isUnder");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "green_cube") != res.end());

  // ReasonerChain

  res = onto_ptr->individuals.getOn("green_cube", "isIn");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "big_box") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerChain"));

  res = onto_ptr->individuals.getOn("blue_cube", "isIn");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "big_box") != res.end());

  // ReasonerDictionary

  res = onto_ptr->individuals.find("big box");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "big_box") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerDictionary"));

  res = onto_ptr->individuals.find("big box");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "big_box") != res.end());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_reasoning_mechanism_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
