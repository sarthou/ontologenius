#include <sstream>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(requests_tests, getName_call)
{
  std::string res;
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.getName(test_word);
    res_bool = res_bool && (res == "human");
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, find_call)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.find(test_word);
    res_bool = res_bool && ((res.size() == 1) && (res[0] == "human"));
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, getUp_call)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.getUp(test_word);
    res_bool = res_bool && ((res.size() == 8) &&
                            (find(res.begin(), res.end(), "entity") != res.end()) &&
                            (find(res.begin(), res.end(), "animate") != res.end()) &&
                            (find(res.begin(), res.end(), "activity") != res.end()) &&
                            (find(res.begin(), res.end(), "attribute") != res.end()) &&
                            (find(res.begin(), res.end(), "human") != res.end()) &&
                            (find(res.begin(), res.end(), "agent") != res.end()) &&
                            (find(res.begin(), res.end(), "vitality") != res.end()) &&
                            (find(res.begin(), res.end(), "living") != res.end()));
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, getDown_call)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.getDown(test_word);
    res_bool = res_bool && ((res.size() == 4) &&
                            (find(res.begin(), res.end(), "human") != res.end()) &&
                            (find(res.begin(), res.end(), "woman") != res.end()) &&
                            (find(res.begin(), res.end(), "man") != res.end()) &&
                            (find(res.begin(), res.end(), "child") != res.end()));
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, getDisjoint_call)
{
  std::vector<std::string> res;
  std::string test_word = "woman";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.getDisjoint(test_word);
    res_bool = res_bool && ((res.size() == 63) &&
                            (find(res.begin(), res.end(), "man") != res.end()));
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, depth_call)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->classes.getUp(test_word, 1);
    res_bool = res_bool && ((res.size() == 3) &&
                            (find(res.begin(), res.end(), "human") != res.end()) &&
                            (find(res.begin(), res.end(), "agent") != res.end()) &&
                            (find(res.begin(), res.end(), "living") != res.end()));
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, select_true_call)
{
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res_bool = res_bool && onto_ptr->classes.isA(test_word, "entity");
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, reasoners_list_call)
{
  std::vector<std::string> res;
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res = onto_ptr->reasoners.list();
    res_bool = res_bool && ((res.size() >= 7) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerChain") != res.end()) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerDictionary") != res.end()) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerInverseOf") != res.end()) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerNone") != res.end()) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerSymmetric") != res.end()) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerGeneralize") != res.end()) &&
                            (find(res.begin(), res.end(), "ontologenius::ReasonerRangeDomain") != res.end()));
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, reasoner_description_call)
{
  std::string res;
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
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
  }

  EXPECT_TRUE(res_bool);
}

TEST(requests_tests, select_false_call)
{
  std::string test_word = "human";
  bool res_bool = true;

  for(size_t i = 0; i < 10; i++)
  {
    res_bool = res_bool && (!onto_ptr->classes.isA(test_word, "animal"));
  }

  EXPECT_TRUE(res_bool);
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
