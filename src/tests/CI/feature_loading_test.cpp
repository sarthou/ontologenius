#include <algorithm>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(feature_loading, reset)
{
  std::vector<std::string> res;

  res = onto_ptr->classes.find("affair");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "affair");

  res = onto_ptr->classes.find("centimeter");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "centimeter");

  EXPECT_TRUE(onto_ptr->actions.clear());
  EXPECT_TRUE(onto_ptr->actions.close());

  res = onto_ptr->classes.find("affair");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "affair");

  res = onto_ptr->classes.find("centimeter");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "centimeter");

  EXPECT_TRUE(onto_ptr->actions.clear());
  std::string path = ros::package::getPath("ontologenius");
  path += "/files/attribute.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  res = onto_ptr->classes.find("affair");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "affair");

  res = onto_ptr->classes.find("centimeter");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "centimeter");
}

TEST(feature_loading, language)
{
  std::vector<std::string> res;

  EXPECT_TRUE(onto_ptr->actions.clear());
  std::string path = ros::package::getPath("ontologenius");
  path += "/files/attribute.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  EXPECT_TRUE(onto_ptr->actions.setLang("en"));

  std::string test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "affair");

  EXPECT_TRUE(onto_ptr->classes.getName(test_word) == "affair");
  EXPECT_FALSE(onto_ptr->classes.getName(test_word) == "affaire");

  EXPECT_TRUE(onto_ptr->actions.setLang("fr"));

  res = onto_ptr->classes.find(test_word);
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "affair");

  test_word = "affaire";
  res = onto_ptr->classes.find(test_word);
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "affair");

  test_word = "affair";
  EXPECT_TRUE(onto_ptr->classes.getName(test_word) == "affaire");
  EXPECT_FALSE(onto_ptr->classes.getName(test_word) == "affair");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_feature_loading_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
