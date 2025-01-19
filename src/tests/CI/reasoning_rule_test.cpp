#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_rule, class_predicate)
{
  // Rule used : Agent(?a) -> isManipulable(?a, boolean#false)

  std::vector<std::string> res;
  onto_ptr->feeder.addConcept("pepper");
  onto_ptr->feeder.addInheritage("pepper", "Robot"); // adding inheritance (indiv isA Class) relation needed for rule inference

  res = onto_ptr->individuals.getOn("pepper", "isManipulable");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addInheritage("Robot", "Agent"); // adding inheritance (Class isA Class) relation needed for rule inference
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getFrom("isManipulable", "boolean#false");
  EXPECT_FALSE(res.empty());

  res = onto_ptr->individuals.getOn("pepper", "isManipulable");
  EXPECT_FALSE(res.empty());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "boolean#false") != res.end()); // checking if the rule inference has created the right data relation

  EXPECT_TRUE(onto_ptr->individuals.isInferred("pepper", "isManipulable", "boolean#false")); // check if the relation has been inferred

  auto exp = onto_ptr->individuals.getInferenceExplanation("pepper", "isManipulable", "boolean#false"); // check the explanations for such inference
  EXPECT_EQ(exp.size(), 1);

  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper|isA|Robot") != res.end());

  onto_ptr->feeder.removeInheritage("pepper", "Robot"); // removing relation used to infer pepper|isManipulable|boolean#false
  onto_ptr->feeder.waitUpdate(1000);
  res = onto_ptr->individuals.getOn("pepper", "isManipulable");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.addInheritage("pepper", "Robot");   // re-adding it
  onto_ptr->feeder.removeInheritage("Robot", "Agent"); // removing class inheritance relation used to infer pepper|isManipulable|boolean#false
  onto_ptr->feeder.waitUpdate(1000);
  res = onto_ptr->individuals.getOn("pepper", "isManipulable");
  EXPECT_TRUE(res.empty());
}

TEST(reasoning_rule, object_predicate)
{
  // Rule used is : Agent(?a), hasCapability(?a,?c) -> Capability(?c)

  std::vector<std::string> res;
  onto_ptr->feeder.addConcept("pepper");
  onto_ptr->feeder.addConcept("pepper_capa");
  onto_ptr->feeder.addInheritage("pepper", "Robot");
  onto_ptr->feeder.addInheritage("Robot", "Agent");

  onto_ptr->feeder.addRelation("pepper", "hasCapability", "pepper_capa");

  res = onto_ptr->individuals.getUp("pepper_capa");
  EXPECT_TRUE(res.empty());

  onto_ptr->feeder.waitUpdate(1000);

  EXPECT_TRUE(onto_ptr->individuals.isInferred("pepper_capa", "Capability"));

  auto exp = onto_ptr->individuals.getInferenceExplanation("pepper_capa", "Capability");
  EXPECT_EQ(exp.size(), 2);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper|isA|Robot") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper|hasCapability|pepper_capa") != res.end());

  onto_ptr->feeder.removeRelation("pepper", "hasCapability", "pepper_capa");
  onto_ptr->feeder.waitUpdate(1000);
  res = onto_ptr->individuals.getUp("pepper_capa");
  EXPECT_TRUE(res.empty());
}

TEST(reasoning_rule, data_predicate)
{
  // Rule used is : Object(?o), isFilled(?o, false) -> isManipulable(?o, true))

  std::vector<std::string> res;
  onto_ptr->feeder.addConcept("mug_3");
  onto_ptr->feeder.addInheritage("mug_3", "Mug");

  onto_ptr->feeder.addRelation("mug_3", "isFilled", "boolean#false");

  onto_ptr->feeder.waitUpdate(1000);

  EXPECT_TRUE(onto_ptr->individuals.isInferred("mug_3", "isManipulable", "boolean#true"));

  auto exp = onto_ptr->individuals.getInferenceExplanation("mug_3", "isManipulable", "boolean#true");
  EXPECT_EQ(exp.size(), 2);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "mug_3|isA|Mug") != exp.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "mug_3|isFilled|boolean#false") != exp.end());

  onto_ptr->feeder.removeRelation("mug_3", "isFilled", "boolean#false");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getOn("mug_3", "isManipulable");
  EXPECT_FALSE(std::find(res.begin(), res.end(), "boolean#true") != res.end());
}

TEST(reasoning_rule, complex_class_predicate)
{
  // Rule used : (Agent and hasCapability some ManipulationCapability)(?a) -> ManipulationCapableAgent(?a)
  // Equivalence class : ManipulationCapability eq to hasAvailableComponent some Gripper
  // Property Chain : isCapabilityOf o hasComponent => hasAvailableComponent

  std::vector<std::string> res;
  onto_ptr->feeder.addConcept("pepper");
  onto_ptr->feeder.addInheritage("pepper", "Robot");
  onto_ptr->feeder.addInheritage("Robot", "Agent");

  onto_ptr->feeder.addConcept("pepper_capa");

  onto_ptr->feeder.addConcept("pepper_left_hand");
  onto_ptr->feeder.addInheritage("pepper_left_hand", "Gripper");

  onto_ptr->feeder.addRelation("pepper", "hasCapability", "pepper_capa");
  onto_ptr->feeder.addRelation("pepper", "hasComponent", "pepper_left_hand");

  onto_ptr->feeder.waitUpdate(1000);

  EXPECT_TRUE(onto_ptr->individuals.isInferred("pepper_capa", "ManipulationCapability"));

  EXPECT_TRUE(onto_ptr->individuals.isInferred("pepper", "ManipulationCapableAgent"));

  auto exp = onto_ptr->individuals.getInferenceExplanation("pepper_capa", "ManipulationCapability");
  EXPECT_EQ(exp.size(), 2);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper_capa|hasAvailableComponent|pepper_left_hand") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper_left_hand|isA|Gripper") != res.end());

  auto exp_manip = onto_ptr->individuals.getInferenceExplanation("pepper", "ManipulationCapableAgent");
  EXPECT_EQ(exp_manip.size(), 4);
  EXPECT_TRUE(std::find(exp_manip.begin(), exp_manip.end(), "pepper_capa|isA|ManipulationCapability") != res.end());
  EXPECT_TRUE(std::find(exp_manip.begin(), exp_manip.end(), "pepper|hasCapability|pepper_capa") != res.end());
  EXPECT_TRUE(std::find(exp_manip.begin(), exp_manip.end(), "pepper|isA|Agent") != res.end()); // should be pepper|isA|Robot -> but is : pepper|isA|Agent
  EXPECT_TRUE(std::find(exp_manip.begin(), exp_manip.end(), "Robot|isA|Agent") != res.end());

  onto_ptr->feeder.removeRelation("pepper", "hasComponent", "pepper_left_hand");
  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_FALSE(onto_ptr->individuals.isA("pepper", "ManipulationCapableAgent"));

  EXPECT_FALSE(onto_ptr->individuals.isA("pepper_capa", "ManipulationCapability"));
}

TEST(reasoning_rule, predicate_combinations)
{
  // Rule is : Robot(?r), hasCapability(?r, ?c), ManipulationCapability(?c), Object(?o),
  //           hasDisposition(?o, ?d), ManipulableDisposition(?d) -> canManipulate(?r, ?o), at

  std::vector<std::string> res;
  onto_ptr->feeder.addConcept("pepper");
  onto_ptr->feeder.addInheritage("pepper", "Robot");
  onto_ptr->feeder.addConcept("pepper_capa");
  onto_ptr->feeder.addRelation("pepper", "hasCapability", "pepper_capa");

  onto_ptr->feeder.addConcept("pepper_left_hand");
  onto_ptr->feeder.addInheritage("pepper_left_hand", "Gripper");

  onto_ptr->feeder.addRelation("pepper", "hasComponent", "pepper_left_hand");

  onto_ptr->feeder.addConcept("mug_3");
  onto_ptr->feeder.addInheritage("mug_3", "Mug");
  onto_ptr->feeder.addConcept("mug_3_disp");
  onto_ptr->feeder.addRelation("mug_3", "hasDisposition", "mug_3_disp");

  onto_ptr->feeder.addRelation("mug_3", "hasPart", "mug_3_handle");
  onto_ptr->feeder.addRelation("mug_3_handle", "isManipulable", "boolean#true");

  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(onto_ptr->individuals.isInferred("pepper", "canManipulate", "mug_3"));

  auto exp = onto_ptr->individuals.getInferenceExplanation("pepper", "canManipulate", "mug_3");
  EXPECT_EQ(exp.size(), 6);
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "mug_3_disp|isA|ManipulableDisposition") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "mug_3|hasDisposition|mug_3_disp") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "mug_3|isA|Mug") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper_capa|isA|ManipulationCapability") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper|hasCapability|pepper_capa") != res.end());
  EXPECT_TRUE(std::find(exp.begin(), exp.end(), "pepper|isA|Robot") != res.end());

  onto_ptr->feeder.removeRelation("mug_3_handle", "isManipulable", "boolean#true");
  onto_ptr->feeder.addRelation("mug_3_handle", "isManipulable", "boolean#false");

  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_FALSE(onto_ptr->individuals.isInferred("pepper", "canManipulate", "mug_3")); // this relation should not exist since we changed the boolean value to false
  onto_ptr->feeder.addRelation("mug_3_handle", "isManipulable", "boolean#true");

  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(onto_ptr->individuals.isInferred("pepper", "canManipulate", "mug_3")); // now it should be true again since we didn't state that the isManipulable property can't point both to false and true

  onto_ptr->feeder.addRelation("robot_helper", "=", "pepper");

  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(onto_ptr->individuals.isInferred("robot_helper", "canManipulate", "mug_3"));
  auto exp_same = onto_ptr->individuals.getInferenceExplanation("robot_helper", "canManipulate", "mug_3");
  EXPECT_EQ(exp_same.size(), 6);
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "mug_3_disp|isA|ManipulableDisposition") != res.end());
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "mug_3|hasDisposition|mug_3_disp") != res.end());
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "mug_3|isA|Mug") != res.end());
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "pepper_capa|isA|ManipulationCapability") != res.end());
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "pepper|hasCapability|pepper_capa") != res.end());
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "pepper|isA|Robot") != res.end());
  EXPECT_TRUE(std::find(exp_same.begin(), exp_same.end(), "robot_helper|sameAs|pepper") != res.end());
}

TEST(reasoning_rule, builtin_tests)
{
  std::vector<std::string> res;
  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addInheritage("a", "Agent");

  onto_ptr->feeder.addRelation("a", "hasAge", "integer#35");
  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(onto_ptr->individuals.isA("a", "Adult"));

  onto_ptr->feeder.removeRelation("a", "hasAge", "integer#35");
  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_FALSE(onto_ptr->individuals.isA("a", "Adult"));

  onto_ptr->feeder.addRelation("a", "hasAge", "integer#12");
  onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(onto_ptr->individuals.isA("a", "Child"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_reasoning_rule_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto_ptr->reasoners.activate("ontologenius::ReasonerRule");
  onto.close();
  onto.feeder.waitConnected();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
