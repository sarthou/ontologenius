#include "ros/ros.h"
#include <gtest/gtest.h>

#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator* onto_ptr;

TEST(library_tests, close_call)
{
  bool res_bool;

  res_bool = onto_ptr->close();
  EXPECT_TRUE(res_bool);
}

TEST(library_tests, individuals_call)
{
  std::vector<std::string> res;
  std::string test_word = "test";
  bool res_bool = true;

  for(size_t i = 0; i < 1000; i++)
  {
    res = onto_ptr->individuals.getUp(test_word);
    res_bool = res_bool && ((res.size() != 1) || (res[0] != "ERR:SERVICE_FAIL"));
  }

  EXPECT_TRUE(res_bool);
}

TEST(library_tests, objectProperties_call)
{
  std::vector<std::string> res;
  std::string test_word = "test";
  bool res_bool = true;

  for(size_t i = 0; i < 1000; i++)
  {
    res = onto_ptr->objectProperties.getUp(test_word);
    res_bool = res_bool && ((res.size() != 1) || (res[0] != "ERR:SERVICE_FAIL"));
  }

  EXPECT_TRUE(res_bool);
}

TEST(library_tests, dataProperties_call)
{
  std::vector<std::string> res;
  std::string test_word = "test";
  bool res_bool = true;

  for(size_t i = 0; i < 1000; i++)
  {
    res = onto_ptr->dataProperties.getUp(test_word);
    res_bool = res_bool && ((res.size() != 1) || (res[0] != "ERR:SERVICE_FAIL"));
  }

  EXPECT_TRUE(res_bool);
}

TEST(library_tests, classes_call)
{
  std::vector<std::string> res;
  std::string test_word = "test";
  bool res_bool = true;

  for(size_t i = 0; i < 1000; i++)
  {
    res = onto_ptr->classes.getUp(test_word);
    res_bool = res_bool && ((res.size() != 1) || (res[0] != "ERR:SERVICE_FAIL"));
  }

  EXPECT_TRUE(res_bool);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_library_tester");

  ros::NodeHandle n;
  OntologyManipulator onto(&n);
  onto_ptr = &onto;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}
