#include "ros/ros.h"
#include <gtest/gtest.h>
#include <ros/package.h>

#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator* onto_ptr;

TEST(global_tests, class_getRelationFrom)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  bool res_bool = true;

  res = onto_ptr->classes.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "hasFather") != res.end()) &&
                          (find(res.begin(), res.end(), "hasMother") != res.end()) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "man";
  res = onto_ptr->classes.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "hasFather") != res.end()) &&
                          (find(res.begin(), res.end(), "hasMother") != res.end()) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "cube";
  res = onto_ptr->classes.getRelationFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelatedFrom)
{
  std::vector<std::string> res;
  std::string test_word = "hasLeg";
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "hasParent";
  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "hasMother";
  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "isOn";
  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelationOn)
{
  std::vector<std::string> res;
  std::string test_word = "woman";
  bool res_bool = true;

  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "hasMother") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "2";
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "0";
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "human";
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelatedOn)
{
  std::vector<std::string> res;
  std::string test_word = "hasLeg";
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "integer:0") != res.end()) &&
                          (find(res.begin(), res.end(), "integer:2") != res.end()));

  test_word = "hasMother";
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "hasFather";
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "man") != res.end()));

  test_word = "hasParent";
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "isOn";
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelationWith)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  bool res_bool = true;

  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "integer:2") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "child";
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "integer:2") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "man";
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "integer:0") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "cube";
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelatedWith)
{
  std::vector<std::string> res;
  std::string test_word = "woman";
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "man";
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "2";
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "0";
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "man") != res.end()));

  test_word = "human";
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getFrom)
{
  std::vector<std::string> res;
  std::string test_word = "woman";
  std::string test_word2 = "hasMother";
  bool res_bool = true;

  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), "child") != res.end()) &&
              (find(res.begin(), res.end(), "man") != res.end()) &&
              (find(res.begin(), res.end(), "human") != res.end()) &&
              (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "woman";
  test_word2 = "hasParent";
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), "child") != res.end()) &&
              (find(res.begin(), res.end(), "man") != res.end()) &&
              (find(res.begin(), res.end(), "human") != res.end()) &&
              (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "2";
  test_word2 = "hasLeg";
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "child") != res.end()) &&
              (find(res.begin(), res.end(), "human") != res.end()) &&
              (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "0";
  test_word2 = "hasLeg";
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "man";
  test_word2 = "hasMother";
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getOn)
{
  std::vector<std::string> res;
  std::string test_word = "man";
  std::string test_word2 = "hasLeg";
  bool res_bool = true;

  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "integer:0") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "child";
  test_word2 = "hasLeg";
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "integer:2") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "woman";
  test_word2 = "hasFather";
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "human";
  test_word2 = "hasMother";
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "human";
  test_word2 = "hasParent";
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
             (find(res.begin(), res.end(), "woman") != res.end()) &&
             (find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "cube";
  test_word2 = "isOn";
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getWith)
{
  std::vector<std::string> res;
  std::string test_word = "human";
  std::string test_word2 = "man";
  bool res_bool = true;

  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "hasFather") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "man";
  test_word2 = "man";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "hasFather") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "child";
  test_word2 = "2";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "man";
  test_word2 = "0";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "human";
  test_word2 = "human";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && (res.size() == 0);

  test_word = "man";
  test_word2 = "2";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
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
