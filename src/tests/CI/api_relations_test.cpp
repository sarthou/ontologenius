#include <algorithm>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(api_relations, class_getRelationFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelationFrom("Human");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasLeg") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationFrom("Man");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasMother") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasLeg") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationFrom("cube");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getRelatedFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedFrom("hasLeg");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedFrom("hasParent");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedFrom("hasMother");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedFrom("isOn");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getRelationOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelationOn("Woman");
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

  res = onto_ptr->classes.getRelationOn("Human");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getRelatedOn)
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
  res_bool = (std::find(res.begin(), res.end(), "Woman") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("hasFather");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "Man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedOn("isOn");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getRelationWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelationWith("Human");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#2") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationWith("Child");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#2") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationWith("Man");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#0") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelationWith("cube");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getRelatedWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedWith("Woman");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("Man");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("integer#2");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "Man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getRelatedWith("Human");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getFrom("hasMother", "Woman");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasParent", "Woman");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasLeg", "integer#2");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "Child") != res.end()) &&
              (std::find(res.begin(), res.end(), "Human") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasLeg", "integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "Man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getFrom("hasMother", "Man");
  res_bool = (res.empty());
  EXPECT_TRUE(res_bool);
}

TEST(api_relations, class_getOn)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getOn("Man", "hasLeg");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "integer#0") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("Child", "hasLeg");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "integer#2") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("Woman", "hasFather");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "Man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("Human", "hasMother");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "Woman") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("Human", "hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "Woman") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getOn("cube", "isOn");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, class_getWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->classes.getWith("Human", "Man");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("Man", "Man");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("Child", "integer#2");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("Man", "integer#0");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "hasLeg") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->classes.getWith("Human", "Human");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->classes.getWith("Man", "integer#2");
  EXPECT_TRUE(res.empty());
}

/*******************
 *
 * INDIVIDUALS
 *
 ********************/

TEST(api_relations, individual_getRelationFrom)
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

  res = onto_ptr->individuals.getRelationFrom("Man");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, individual_getRelatedFrom)
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
              (std::find(res.begin(), res.end(), "green_cube") != res.end()) &&
              (std::find(res.begin(), res.end(), "blue_cube") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedFrom("Human");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, individual_getRelationOn)
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

  res = onto_ptr->individuals.getRelationOn("Human");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, individual_getRelatedOn)
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

TEST(api_relations, individual_getRelationWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationWith("alice");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "integer#2") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
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
              (std::find(res.begin(), res.end(), "Man") != res.end()) &&
              (std::find(res.begin(), res.end(), "Woman") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelationWith("cube1");
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), "integer#236") != res.end()) &&
              (std::find(res.begin(), res.end(), "big_box") != res.end()) &&
              (std::find(res.begin(), res.end(), "blue_cube") != res.end()) &&
              (std::find(res.begin(), res.end(), "red_cube") != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(api_relations, individual_getRelatedWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedWith("Woman");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getRelatedWith("Man");
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

  res = onto_ptr->individuals.getRelatedWith("Human");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, individual_getFrom)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getFrom("hasMother", "Woman");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasParent", "Woman");
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), "kevin") != res.end()) &&
              (std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getFrom("hasParent", "Human");
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

  res = onto_ptr->individuals.getFrom("hasMother", "Man");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, individual_getOn)
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
  res_bool = (std::find(res.begin(), res.end(), "Man") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("bob", "hasMother");
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), "Woman") != res.end());
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("alice", "hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "Woman") != res.end()) &&
              (std::find(res.begin(), res.end(), "Man") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("kevin", "hasParent");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "alice") != res.end()) &&
              (std::find(res.begin(), res.end(), "bob") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getOn("Man", "hasLeg");
  EXPECT_TRUE(res.empty());
}

TEST(api_relations, individual_getWith)
{
  std::vector<std::string> res;
  bool res_bool = true;

  res = onto_ptr->individuals.getWith("kevin", "bob");
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "hasFather") != res.end()) &&
              (std::find(res.begin(), res.end(), "hasParent") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->individuals.getWith("bob", "Man");
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

  res = onto_ptr->individuals.getWith("alice", "Human");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getWith("bob", "integer#2");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getWith("cube1", "red_cube"); // use same as (cube1 = green_cube)
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), "isPositioned") != res.end()) &&
              (std::find(res.begin(), res.end(), "isOn") != res.end()));
  EXPECT_TRUE(res_bool);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_api_relations_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
