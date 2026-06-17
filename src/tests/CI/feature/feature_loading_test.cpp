#include <algorithm>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"
#include "ontologenius/utils/Commands.h"

onto::OntologyManipulator* onto_ptr;

TEST(feature_loading, reset)
{
  std::vector<std::string> res;

  res = onto_ptr->classes.find("affair");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "Affair");

  res = onto_ptr->classes.find("bird");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "Bird");

  EXPECT_TRUE(onto_ptr->actions.clear());
  EXPECT_TRUE(onto_ptr->actions.close());

  res = onto_ptr->classes.find("affair");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "Affair");

  res = onto_ptr->classes.find("bird");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "Bird");

  EXPECT_TRUE(onto_ptr->actions.clear());
  std::string path = ontologenius::findPackage("ontologenius");
  path += "/files/objects.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  res = onto_ptr->classes.find("affair");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "Affair");

  res = onto_ptr->classes.find("bird");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "Bird");
}

TEST(feature_loading, language)
{
  std::vector<std::string> res;

  EXPECT_TRUE(onto_ptr->actions.clear());
  std::string path = ontologenius::findPackage("ontologenius");
  path += "/files/objects.owl";
  EXPECT_TRUE(onto_ptr->actions.fadd(path));

  EXPECT_TRUE(onto_ptr->actions.close());

  EXPECT_TRUE(onto_ptr->actions.setLang("en"));

  std::string test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "Affair");

  EXPECT_TRUE(onto_ptr->classes.getName("Affair") == "affair");
  EXPECT_FALSE(onto_ptr->classes.getName("Affair") == "affaire");

  EXPECT_TRUE(onto_ptr->actions.setLang("fr"));

  res = onto_ptr->classes.find("Affair");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "Affair");

  res = onto_ptr->classes.find("affaire");
  EXPECT_EQ(res.size(), 1);
  if(res.empty() == false)
    EXPECT_EQ(res.front(), "Affair");

  EXPECT_TRUE(onto_ptr->classes.getName("Affair") == "affaire");
  EXPECT_FALSE(onto_ptr->classes.getName("Affair") == "Affair");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  int res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
