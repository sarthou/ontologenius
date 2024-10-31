#include <algorithm>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(global_tests, class_getRelationFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelationFrom("human");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasLeg") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationFrom("man");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasLeg") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationFrom("cube");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getRelatedFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedFrom("hasLeg");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedFrom("hasParent");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedFrom("hasMother");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedFrom("isOn");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getRelationOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelationOn("woman");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationOn("integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationOn("integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationOn("human");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getRelatedOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedOn("hasLeg");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (std::find(res.begin(), res.end(), "integer#2") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("hasMother");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "woman") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("hasFather");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("isOn");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getRelationWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelationWith("human");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#2") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationWith("child");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#2") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationWith("man");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationWith("cube");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getRelatedWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedWith("woman");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("man");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("integer#2");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("human");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getFrom("hasMother", "woman");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasParent", "woman");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasLeg", "integer#2");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "child") != res.end()) &&
              (std::find(res.begin(), res.end(), "human") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasLeg", "integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasMother", "man");
  res_bool = (res.empty());
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getOn("man", "hasLeg");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "integer#0") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("child", "hasLeg");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "integer#2") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("woman", "hasFather");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("human", "hasMother");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "woman") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("human", "hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "woman") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("cube", "isOn");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, class_getWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getWith("human", "man");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("man", "man");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("child", "integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("man", "integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("human", "human");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->classes.getWith("man", "integer#2");
  EXPECT_TRUE(res.empty());
}

/*******************
 *
 * INDIVIDUALS
 *
 ********************/

TEST(global_tests, individual_getRelationFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationFrom("alice");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasLeg") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationFrom("kevin");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasLeg") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationFrom("cube1");
  EXPECT_EQ(res.size(), 5);
  res_bool = ((std::find(res.begin(), res.end(), "isPositioned") != res.end()) &&
              (std::find(res.begin(), res.end(), "isOn") != res.end()) &&
              (std::find(res.begin(), res.end(), "objectHave3Dposition") != res.end()) &&
              (std::find(res.begin(), res.end(), "isIn") != res.end()) &&
              (std::find(res.begin(), res.end(), "have3Dposition") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationFrom("man");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, individual_getRelatedFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedFrom("hasLeg");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedFrom("hasParent");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedFrom("isOn");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "cube1") != res.end()) &&
              (std::find(res.begin(), res.end(), "mini_box") != res.end()) &&
              (std::find(res.begin(), res.end(), "greenCube") != res.end()) &&
              (std::find(res.begin(), res.end(), "blueCube") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedFrom("human");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, individual_getRelationOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationOn("alice");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationOn("cube1");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "isUnder") != res.end()) &&
              (std::find(res.begin(), res.end(), "isPositioned") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationOn("integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationOn("integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationOn("human");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, individual_getRelatedOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedOn("hasLeg");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (std::find(res.begin(), res.end(), "integer#2") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedOn("hasMother");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "alice") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedOn("hasFather");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "bob") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedOn("hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "bob") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedOn("isIn");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "big_box") != res.end());
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelationWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationWith("alice");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#2") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationWith("kevin");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationWith("bob");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()) &&
              (std::find(res.begin(), res.end(), "woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationWith("cube1");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "integer#236") != res.end()) &&
              (std::find(res.begin(), res.end(), "big_box") != res.end()) &&
              (std::find(res.begin(), res.end(), "blueCube") != res.end()) &&
              (std::find(res.begin(), res.end(), "redCube") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedWith("woman");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedWith("man");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedWith("integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "alice") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedWith("integer#0");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedWith("human");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, individual_getFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getFrom("hasMother", "woman");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasParent", "woman");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasParent", "human");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasLeg", "integer#0");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasLeg", "integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "alice") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasMother", "man");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, individual_getOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getOn("bob", "hasLeg");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "integer#0") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("kevin", "hasLeg");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "integer#0") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("alice", "hasFather");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("bob", "hasMother");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "woman") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("alice", "hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "woman") != res.end()) &&
              (std::find(res.begin(), res.end(), "man") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("kevin", "hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("man", "hasLeg");
  EXPECT_TRUE(res.empty());
}

TEST(global_tests, individual_getWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getWith("kevin", "bob");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getWith("bob", "man");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getWith("alice", "integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getWith("bob", "integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getWith("alice", "human");
  EXPECT_TRUE(res.empty());

  test_word = "bob";
  test_word2 = "integer#2";
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getWith("cube1", "redCube"); // use same as (cube1 = greenCube)
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "isPositioned") != res.end()) &&
              (std::find(res.begin(), res.end(), "isOn") != res.end()));
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
