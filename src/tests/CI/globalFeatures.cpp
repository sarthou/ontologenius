#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(global_tests, reset)
{
  std::vector<std::string> res;

  std::string test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "affair"));

  test_word = "centimeter";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "centimeter"));

  EXPECT_TRUE(onto_ptr->actions.clear());
  EXPECT_TRUE(onto_ptr->actions.close());

  test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_FALSE((res.size() == 1) && (res[0] == "affair"));

  test_word = "centimeter";
  res = onto_ptr->classes.find(test_word);
  EXPECT_FALSE((res.size() == 1) && (res[0] == "centimeter"));

  EXPECT_TRUE(onto_ptr->actions.clear());
  std::string path = ros::package::getPath("ontologenius");
  path+= "/files/attribute.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "affair"));

  test_word = "centimeter";
  res = onto_ptr->classes.find(test_word);
  EXPECT_FALSE((res.size() == 1) && (res[0] == "centimeter"));
}

TEST(global_tests, language)
{
  std::vector<std::string> res;

  EXPECT_TRUE(onto_ptr->actions.clear());
  std::string path = ros::package::getPath("ontologenius");
  path+= "/files/attribute.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  EXPECT_TRUE(onto_ptr->actions.setLang("en"));

  std::string test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "affair"));

  EXPECT_TRUE(onto_ptr->classes.getName(test_word) == "affair");
  EXPECT_FALSE(onto_ptr->classes.getName(test_word) == "affaire");

  EXPECT_TRUE(onto_ptr->actions.setLang("fr"));

  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "affair"));

  test_word = "affaire";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "affair"));

  test_word = "affair";
  EXPECT_TRUE(onto_ptr->classes.getName(test_word) == "affaire");
  EXPECT_FALSE(onto_ptr->classes.getName(test_word) == "affair");
}

TEST(global_tests, reasoners_effect)
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

  path = path_base + "/files/testIndividuals.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  //ReasonerSymmetric

  res = onto_ptr->individuals.getOn("redCube", "isInFrontOf");
  EXPECT_TRUE(find(res.begin(), res.end(), "blue_book") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerSymmetric"));

  res = onto_ptr->individuals.getOn("redCube", "isInFrontOf");
  EXPECT_TRUE(find(res.begin(), res.end(), "blue_book") != res.end());

  //ReasonerInverseOf

  res = onto_ptr->individuals.getOn("blueCube", "isUnder");
  EXPECT_TRUE(find(res.begin(), res.end(), "greenCube") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerInverseOf"));

  res = onto_ptr->individuals.getOn("blueCube", "isUnder");
  EXPECT_TRUE(find(res.begin(), res.end(), "greenCube") != res.end());

  //ReasonerChain

  res = onto_ptr->individuals.getOn("greenCube", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "big_box") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerChain"));

  res = onto_ptr->individuals.getOn("blueCube", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "big_box") != res.end());

  //ReasonerDictionary

  res = onto_ptr->individuals.find("big box");
  EXPECT_TRUE(find(res.begin(), res.end(), "big_box") == res.end());

  EXPECT_TRUE(onto_ptr->reasoners.activate("ontologenius::ReasonerDictionary"));

  res = onto_ptr->individuals.find("big box");
  EXPECT_TRUE(find(res.begin(), res.end(), "big_box") != res.end());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_global_tester");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
