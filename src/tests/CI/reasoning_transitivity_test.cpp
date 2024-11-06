#include <algorithm>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

#define WAIT_TIME 1000

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_transitivity, transitivity_base)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "topTransitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addProperty("c", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());
}

TEST(reasoning_transitivity, transitivity_heritance)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "transitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "transitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addProperty("c", "transitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());
}

TEST(reasoning_transitivity, transitivity_sames)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "=", "b_bis");
  onto_ptr->feeder.addProperty("b_bis", "topTransitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addProperty("c", "=", "c_bis");
  onto_ptr->feeder.addProperty("c_bis", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());
}

TEST(reasoning_transitivity, transitivity_deletion)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "topTransitiveProperty", "c");
  onto_ptr->feeder.addProperty("c", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());
}

TEST(reasoning_transitivity, transitivity_deletion_same_as)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "=", "b_bis");
  onto_ptr->feeder.addProperty("b_bis", "topTransitiveProperty", "c");
  onto_ptr->feeder.addProperty("c", "=", "c_bis");
  onto_ptr->feeder.addProperty("c_bis", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeProperty("b", "=", "b_bis");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_FALSE(std::find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_FALSE(std::find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_FALSE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_FALSE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());
}

TEST(reasoning_transitivity, transitivity_deletion_inheritage)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "transitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "transitiveProperty", "c");
  onto_ptr->feeder.addProperty("c", "transitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeInheritage("transitiveProperty", "topTransitiveProperty");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "transitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_FALSE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_FALSE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("b", "transitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_FALSE(std::find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_reasoning_transitivity_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();
  onto.feeder.waitConnected();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
