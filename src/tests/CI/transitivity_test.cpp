#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(chain_tests, transitivity_base)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "topTransitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addProperty("c", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());
}

TEST(chain_tests, transitivity_heritance)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "transitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "transitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addProperty("c", "transitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());
}

TEST(chain_tests, transitivity_sames)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "=", "b_bis");
  onto_ptr->feeder.addProperty("b_bis", "topTransitiveProperty", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());

  onto_ptr->feeder.addProperty("c", "=", "c_bis");
  onto_ptr->feeder.addProperty("c_bis", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());
}

TEST(chain_tests, transitivity_deletion)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "topTransitiveProperty", "c");
  onto_ptr->feeder.addProperty("c", "topTransitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeProperty("a", "topTransitiveProperty", "b");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());
}

TEST(chain_tests, transitivity_deletion_same_as)
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
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeProperty("b", "=", "b_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_FALSE(find(res.begin(), res.end(), "b_bis") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_FALSE(find(res.begin(), res.end(), "c_bis") != res.end());
  EXPECT_FALSE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_FALSE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());
}

TEST(chain_tests, transitivity_deletion_inheritage)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerTransitivity");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "transitiveProperty", "b");
  onto_ptr->feeder.addProperty("b", "transitiveProperty", "c");
  onto_ptr->feeder.addProperty("c", "transitiveProperty", "d");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_TRUE(find(res.begin(), res.end(), "d") != res.end());

  onto_ptr->feeder.removeInheritage("transitiveProperty", "topTransitiveProperty");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("a", "transitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "b") != res.end());
  EXPECT_FALSE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_FALSE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("a", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("b", "transitiveProperty");
  EXPECT_TRUE(find(res.begin(), res.end(), "c") != res.end());
  EXPECT_FALSE(find(res.begin(), res.end(), "d") != res.end());

  res = onto_ptr->individuals.getOn("b", "topTransitiveProperty");
  EXPECT_TRUE(res.empty());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_transitivity_inference");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();
  onto.feeder.waitConnected();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}
