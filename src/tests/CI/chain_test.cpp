#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(chain_tests, chains_base)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());
}

TEST(chain_tests, chain_heritance)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isInBox");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOnTop", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("ball", "isOnTop", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());
}

TEST(chain_tests, chain_sames)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());
}

TEST(chain_tests, chain_deletion)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOn", "cube_after");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("cube_base", "isIn", "box");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.addProperty("cube_base", "isIn", "box");
  onto_ptr->feeder.removeProperty("ball", "isOn", "cube_after");
  onto_ptr->feeder.waitUpdate(1000);
}

TEST(chain_tests, chain_deletion_same_as)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("cube_base", "=", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("cube_base", "=", "cube_base_bis");
  onto_ptr->feeder.removeProperty("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);
}

TEST(chain_tests, chain_deletion_inheritage)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addProperty("ball", "isOnTop", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeInheritage("isOnTop", "isOn");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addInheritage("isOnTop", "isOn"); // we check if adding again the relation trigger the reasoning
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeProperty("ball", "isOnTop", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_chain_inference");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();
  onto.feeder.waitConnected();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}
