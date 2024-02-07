#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

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

  test_word = "integer#2";
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "integer#0";
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
                          (find(res.begin(), res.end(), "integer#0") != res.end()) &&
                          (find(res.begin(), res.end(), "integer#2") != res.end()));

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
                          (find(res.begin(), res.end(), "integer#2") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "child";
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "integer#2") != res.end()) &&
                          (find(res.begin(), res.end(), "man") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "man";
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "integer#0") != res.end()) &&
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

  test_word = "integer#2";
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "child") != res.end()) &&
                          (find(res.begin(), res.end(), "human") != res.end()) &&
                          (find(res.begin(), res.end(), "woman") != res.end()));

  test_word = "integer#0";
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

  test_word = "integer#2";
  test_word2 = "hasLeg";
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "child") != res.end()) &&
              (find(res.begin(), res.end(), "human") != res.end()) &&
              (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "integer#0";
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
             (find(res.begin(), res.end(), "integer#0") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "child";
  test_word2 = "hasLeg";
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "integer#2") != res.end()));
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
  test_word2 = "integer#2";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "man";
  test_word2 = "integer#0";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "human";
  test_word2 = "human";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && (res.size() == 0);

  test_word = "man";
  test_word2 = "integer#2";
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

/*******************
*
* INDIVIDUALS
*
********************/

TEST(global_tests, individual_getRelationFrom)
{
  std::vector<std::string> res;
  std::string test_word = "alice";
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "hasFather") != res.end()) &&
                          (find(res.begin(), res.end(), "hasMother") != res.end()) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "kevin";
  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "hasFather") != res.end()) &&
                          (find(res.begin(), res.end(), "hasMother") != res.end()) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "cube1";
  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 5) &&
                          (find(res.begin(), res.end(), "isPositioned") != res.end()) &&
                          (find(res.begin(), res.end(), "isOn") != res.end()) &&
                          (find(res.begin(), res.end(), "objectHave3Dposition") != res.end()) &&
                          (find(res.begin(), res.end(), "isIn") != res.end()) &&
                          (find(res.begin(), res.end(), "have3Dposition") != res.end()));

  test_word = "man";
  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedFrom)
{
  std::vector<std::string> res;
  std::string test_word = "hasLeg";
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "kevin") != res.end()) &&
                          (find(res.begin(), res.end(), "alice") != res.end()) &&
                          (find(res.begin(), res.end(), "bob") != res.end()));

  test_word = "hasParent";
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "kevin") != res.end()) &&
                          (find(res.begin(), res.end(), "alice") != res.end()) &&
                          (find(res.begin(), res.end(), "bob") != res.end()));

  test_word = "isOn";
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), "cube1") != res.end()) &&
                          (find(res.begin(), res.end(), "mini_box") != res.end()) &&
                          (find(res.begin(), res.end(), "greenCube") != res.end()) &&
                          (find(res.begin(), res.end(), "blueCube") != res.end()));

  test_word = "human";
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelationOn)
{
  std::vector<std::string> res;
  std::string test_word = "alice";
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "hasMother") != res.end()) &&
                          (find(res.begin(), res.end(), "hasParent") != res.end()));

  test_word = "cube1";
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "isUnder") != res.end()) &&
                          (find(res.begin(), res.end(), "isPositioned") != res.end()));

  test_word = "integer#2";
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "integer#0";
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "hasLeg") != res.end()));

  test_word = "human";
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedOn)
{
  std::vector<std::string> res;
  std::string test_word = "hasLeg";
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "integer#0") != res.end()) &&
                          (find(res.begin(), res.end(), "integer#2") != res.end()));

  test_word = "hasMother";
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "alice") != res.end()));

  test_word = "hasFather";
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "bob") != res.end()));

  test_word = "hasParent";
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "bob") != res.end()) &&
                          (find(res.begin(), res.end(), "alice") != res.end()));

  test_word = "isIn";
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "big_box") != res.end()));

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelationWith)
{
  std::vector<std::string> res;
  std::string test_word = "alice";
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 3) &&
            (find(res.begin(), res.end(), "integer#2") != res.end()) &&
            (find(res.begin(), res.end(), "man") != res.end()) &&
            (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "kevin";
  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (find(res.begin(), res.end(), "bob") != res.end()) &&
              (find(res.begin(), res.end(), "alice") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "bob";
  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (find(res.begin(), res.end(), "man") != res.end()) &&
              (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "cube1";
  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), "integer#236") != res.end()) &&
              (find(res.begin(), res.end(), "big_box") != res.end()) &&
              (find(res.begin(), res.end(), "blueCube") != res.end()) &&
              (find(res.begin(), res.end(), "redCube") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedWith)
{
  std::vector<std::string> res;
  std::string test_word = "woman";
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "kevin") != res.end()) &&
                          (find(res.begin(), res.end(), "alice") != res.end()) &&
                          (find(res.begin(), res.end(), "bob") != res.end()));

  test_word = "man";
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), "kevin") != res.end()) &&
                          (find(res.begin(), res.end(), "alice") != res.end()) &&
                          (find(res.begin(), res.end(), "bob") != res.end()));

  test_word = "integer#2";
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), "alice") != res.end()));

  test_word = "integer#0";
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), "kevin") != res.end()) &&
                          (find(res.begin(), res.end(), "bob") != res.end()));

  test_word = "human";
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getFrom)
{
  std::vector<std::string> res;
  std::string test_word = "woman";
  std::string test_word2 = "hasMother";
  bool res_bool = true;

  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "kevin") != res.end()) &&
              (find(res.begin(), res.end(), "alice") != res.end()) &&
              (find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "woman";
  test_word2 = "hasParent";
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "kevin") != res.end()) &&
              (find(res.begin(), res.end(), "alice") != res.end()) &&
              (find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "human";
  test_word2 = "hasParent";
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "kevin") != res.end()) &&
              (find(res.begin(), res.end(), "alice") != res.end()) &&
              (find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "integer#0";
  test_word2 = "hasLeg";
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "kevin") != res.end()) &&
              (find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "integer#2";
  test_word2 = "hasLeg";
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "alice") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "man";
  test_word2 = "hasMother";
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getOn)
{
  std::vector<std::string> res;
  std::string test_word = "bob";
  std::string test_word2 = "hasLeg";
  bool res_bool = true;

  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "integer#0") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "kevin";
  test_word2 = "hasLeg";
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "integer#0") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "alice";
  test_word2 = "hasFather";
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "bob";
  test_word2 = "hasMother";
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "alice";
  test_word2 = "hasParent";
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
             (find(res.begin(), res.end(), "woman") != res.end()) &&
             (find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "kevin";
  test_word2 = "hasParent";
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
             (find(res.begin(), res.end(), "alice") != res.end()) &&
             (find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "man";
  test_word2 = "hasLeg";
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getWith)
{
  std::vector<std::string> res;
  std::string test_word = "kevin";
  std::string test_word2 = "bob";
  bool res_bool = true;

  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "bob";
  test_word2 = "man";
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "alice";
  test_word2 = "integer#2";
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "hasLeg") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "bob";
  test_word2 = "integer#0";
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "hasLeg") != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = "alice";
  test_word2 = "human";
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);

  test_word = "bob";
  test_word2 = "integer#2";
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);

  test_word = "cube1";
  test_word2 = "redCube";
  res = onto_ptr->individuals.getWith(test_word, test_word2); // use same as (cube1 = greenCube)
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "isPositioned") != res.end()) &&
              (find(res.begin(), res.end(), "isOn") != res.end()));
  EXPECT_TRUE(res_bool);
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
