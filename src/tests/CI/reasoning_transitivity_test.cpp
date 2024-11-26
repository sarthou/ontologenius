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
  onto_ptr->feeder.addRelation("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addRelation("b", "topTransitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());

  EXPECT_FALSE(onto_ptr->individuals.isInferred("a", "topTransitiveProperty", "b"));
  EXPECT_TRUE(onto_ptr->individuals.isInferred("a", "topTransitiveProperty", "c"));
  auto exp = onto_ptr->individuals.getInferenceExplanation("a", "topTransitiveProperty", "c");
  EXPECT_EQ(exp.size(), 2);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "a|topTransitiveProperty|b") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "b|topTransitiveProperty|c") != res.end());

  onto_ptr->feeder.addRelation("c", "topTransitiveProperty", "d");
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
  onto_ptr->feeder.addRelation("a", "transitiveProperty", "b");
  onto_ptr->feeder.addRelation("b", "transitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addRelation("c", "transitiveProperty", "d");
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
  onto_ptr->feeder.addRelation("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addRelation("b", "=", "b_bis");
  onto_ptr->feeder.addRelation("b_bis", "topTransitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addRelation("c", "=", "c_bis");
  onto_ptr->feeder.addRelation("c_bis", "topTransitiveProperty", "d");
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
  onto_ptr->feeder.addRelation("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addRelation("b", "topTransitiveProperty", "c");
  onto_ptr->feeder.addRelation("c", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(WAIT_TIME);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeRelation("a", "topTransitiveProperty", "b");
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
  onto_ptr->feeder.addRelation("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addRelation("b", "=", "b_bis");
  onto_ptr->feeder.addRelation("b_bis", "topTransitiveProperty", "c");
  onto_ptr->feeder.addRelation("c", "=", "c_bis");
  onto_ptr->feeder.addRelation("c_bis", "topTransitiveProperty", "d");
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

  onto_ptr->feeder.removeRelation("b", "=", "b_bis");
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
  onto_ptr->feeder.addRelation("a", "transitiveProperty", "b");
  onto_ptr->feeder.addRelation("b", "transitiveProperty", "c");
  onto_ptr->feeder.addRelation("c", "transitiveProperty", "d");
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
