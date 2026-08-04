#include <algorithm>
#include <gtest/gtest.h>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_anonymous_class, init)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getRelationFrom("uncle", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasMother") != res.end());

  res = onto_ptr->individuals.getOn("uncle", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tata") != res.end());

  res = onto_ptr->individuals.getOn("uncle", "hasMother");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "papa") != res.end());

  res = onto_ptr->individuals.getRelationFrom("bob", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasMother") != res.end());

  res = onto_ptr->individuals.getOn("bob", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tonton") != res.end());

  res = onto_ptr->individuals.getOn("bob", "hasMother");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "mama") != res.end());

  res = onto_ptr->individuals.getUp("dad", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Uncle") != res.end());

  res = onto_ptr->individuals.getRelationFrom("dad", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasMother") != res.end());

  res = onto_ptr->individuals.getOn("dad", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tata") != res.end());

  res = onto_ptr->individuals.getOn("dad", "hasMother");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "papa") != res.end());
}

TEST(reasoning_anonymous_class, equiv_inherit_equiv)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);

  onto_ptr->feeder.addInheritage("a", "Uncle");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 1);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasMother") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tata") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasMother");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "papa") != res.end());

  onto_ptr->feeder.removeInheritage("a", "Uncle");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);
}

TEST(reasoning_anonymous_class, equiv_inherit_sub_class)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);

  onto_ptr->feeder.addInheritage("a", "Bob");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 1);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasMother") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tonton") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasMother");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "mama") != res.end());

  onto_ptr->feeder.removeInheritage("a", "Bob");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);
}

TEST(reasoning_anonymous_class, equiv_deduce_equiv)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);

  onto_ptr->feeder.addRelation("a", "hasSurname", "string#tata");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Uncle") != res.end());

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasMother") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tata") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasMother");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "papa") != res.end());

  onto_ptr->feeder.removeRelation("a", "hasSurname", "string#tata");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);
}

TEST(reasoning_anonymous_class, equiv_not_deduce_subclass)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);

  onto_ptr->feeder.addRelation("a", "hasSurname", "string#tonton");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "hasSurname") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasSurname");
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "string#tonton") != res.end());

  res = onto_ptr->individuals.getOn("a", "hasMother");
  EXPECT_EQ(res.size(), 0);

  onto_ptr->feeder.removeRelation("a", "hasSurname", "string#tonton");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 0);

  res = onto_ptr->individuals.getRelationFrom("a", 1);
  EXPECT_EQ(res.size(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");
  onto.close();
  onto.feeder.waitConnected();
  onto.feeder.waitUpdate(1000);

  int res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
