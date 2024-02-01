#include <ros/ros.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

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

  for(size_t i = 0; i < 10; i++)
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

  for(size_t i = 0; i < 10; i++)
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

  for(size_t i = 0; i < 10; i++)
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

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.getUp(test_word);
    res_bool = res_bool && ((res.size() != 1) || (res[0] != "ERR:SERVICE_FAIL"));
  }

  EXPECT_TRUE(res_bool);
}

TEST(library_tests, actions_call)
{
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res_bool = res_bool && onto_ptr->actions.setLang("en");
  }

  EXPECT_TRUE(res_bool);
}

TEST(library_tests, reasoners_call)
{
  std::vector<std::string> res;
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->reasoners.list();
    res_bool = res_bool && ((res.size() != 1) || (res[0] != "ERR:SERVICE_FAIL"));
  }

  EXPECT_TRUE(res_bool);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_library_tester");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
