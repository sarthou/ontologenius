#include "ros/ros.h"
#include <gtest/gtest.h>
#include <ros/package.h>

#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator* onto_ptr;

TEST(global_tests, reset)
{
  std::vector<std::string> res;

  std::string test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "affair"));

  test_word = "centimeter";
  res = onto_ptr->classes.find(test_word);
  EXPECT_TRUE((res.size() == 1) && (res[0] == "centimeter"));

  EXPECT_TRUE(onto_ptr->actions.reset());
  EXPECT_TRUE(onto_ptr->actions.close());

  test_word = "affair";
  res = onto_ptr->classes.find(test_word);
  EXPECT_FALSE((res.size() == 1) && (res[0] == "affair"));

  test_word = "centimeter";
  res = onto_ptr->classes.find(test_word);
  EXPECT_FALSE((res.size() == 1) && (res[0] == "centimeter"));

  EXPECT_TRUE(onto_ptr->actions.reset());
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_global_tester");

  ros::NodeHandle n;
  OntologyManipulator onto(&n);
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}
