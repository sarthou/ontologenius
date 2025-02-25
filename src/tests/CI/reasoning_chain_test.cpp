#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_chain, chains_base)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "box") != res.end());

  EXPECT_FALSE(onto_ptr->individuals.isInferred("ball", "isOn", "cube_base"));
  EXPECT_TRUE(onto_ptr->individuals.isInferred("ball", "isIn", "box"));
  auto exp = onto_ptr->individuals.getInferenceExplanation("ball", "isIn", "box");
  EXPECT_EQ(exp.size(), 2);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "ball|isOn|cube_base") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "cube_base|isIn|box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeRelation("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());
}

TEST(reasoning_chain, chain_heritance)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isInBox");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOnTop", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  EXPECT_FALSE(onto_ptr->individuals.isInferred("ball", "isOnTop", "cube_base"));
  EXPECT_TRUE(onto_ptr->individuals.isInferred("ball", "isIn", "box"));
  auto exp = onto_ptr->individuals.getInferenceExplanation("ball", "isIn", "box");
  EXPECT_EQ(exp.size(), 3);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "ball|isOnTop|cube_base") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "isOnTop|isA|isOn") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "cube_base|isIn|box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeRelation("ball", "isOnTop", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());
}

TEST(reasoning_chain, chain_sames)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeRelation("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());
}

TEST(reasoning_chain, chain_deletion)
{
  std::vector<std::string> res;
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeRelation("ball", "isOn", "cube_base");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOn", "cube_after");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeRelation("cube_base", "isIn", "box");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.addRelation("cube_base", "isIn", "box");
  onto_ptr->feeder.removeRelation("ball", "isOn", "cube_after");
  onto_ptr->feeder.waitUpdate(1000);
}

TEST(reasoning_chain, chain_deletion_same_as)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(find(res.begin(), res.end(), "box") != res.end());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(find(res.begin(), res.end(), "table") != res.end());

  onto_ptr->feeder.removeRelation("cube_base", "=", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("cube_base", "=", "cube_base_bis");
  onto_ptr->feeder.removeRelation("ball", "isOn", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);
}

TEST(reasoning_chain, chain_deletion_inheritage)
{
  std::vector<std::string> res;
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->reasoners.activate("ontologenius::ReasonerChain");

  res = onto_ptr->individuals.getOn("ball", "isIn");
  EXPECT_TRUE(res.empty());

  res = onto_ptr->individuals.getOn("ball", "thirdChain");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addRelation("ball", "isOnTop", "cube_base_bis");
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

  onto_ptr->feeder.removeRelation("ball", "isOnTop", "cube_base_bis");
  onto_ptr->feeder.waitUpdate(1000);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_reasoning_chain_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();
  onto.feeder.waitConnected();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
