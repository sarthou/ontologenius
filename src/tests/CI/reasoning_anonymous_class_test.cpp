#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_anonymous_class, same_as_range_restriction)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getUp("indiv2");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RealSenseVisionCapability") != res.end());

  res = onto_ptr->individuals.getUp("indiv1"); // indiv1 = indiv2
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RealSenseVisionCapability") != res.end());

  onto_ptr->feeder.addConcept("indiv3");
  onto_ptr->feeder.addRelation("indiv3", "hasCapability", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3"); // indiv3 hasCapability indiv1
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PepperVisionCapability") != res.end());

  onto_ptr->feeder.removeRelation("indiv3", "hasCapability", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PepperVisionCapability") == res.end());

  onto_ptr->feeder.addRelation("indiv3", "hasCapability", "indiv1");
  onto_ptr->feeder.removeRelation("indiv2", "=", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PepperVisionCapability") == res.end());

  onto_ptr->feeder.addRelation("indiv2", "=", "indiv1");
  onto_ptr->feeder.removeRelation("realsense_d435i", "=", "realsense_pepper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PepperVisionCapability") == res.end());
}

TEST(reasoning_anonymous_class, trace_cleaning)
{
  std::vector<std::string> res;

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addRelation("a", "hasComponent", "b");
  onto_ptr->feeder.addRelation("b", "hasCamera", "c");
  onto_ptr->feeder.addInheritage("c", "Camera");
  onto_ptr->feeder.addRelation("b", "hasComponent", "d");
  onto_ptr->feeder.addInheritage("d", "Lidar");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "LocalizeCapability") != res.end());

  onto_ptr->feeder.removeRelation("b", "hasCamera", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "LocalizeCapability") == res.end());

  onto_ptr->feeder.addRelation("b", "hasCamera", "e");
  onto_ptr->feeder.addInheritage("e", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "LocalizeCapability") != res.end());

  onto_ptr->feeder.removeInheritage("c", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "LocalizeCapability") != res.end());

  onto_ptr->feeder.addInheritage("c", "Camera");
  onto_ptr->feeder.removeInheritage("e", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "LocalizeCapability") == res.end());

  onto_ptr->feeder.addRelation("e", "=", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "LocalizeCapability") != res.end());
}

TEST(reasoning_anonymous_class, cardinality_min_testing)
{
  std::vector<std::string> res;

  onto_ptr->feeder.addRelation("pepper", "hasComponent", "bumperLeft");
  onto_ptr->feeder.addRelation("pepper", "hasComponent", "bumperRight");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ObstacleAvoidanceCapability") != res.end());

  onto_ptr->feeder.addConcept("bumperMiddle");
  onto_ptr->feeder.addInheritage("bumperMiddle", "Bumper");
  onto_ptr->feeder.addRelation("pepper", "hasComponent", "bumperMiddle");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ObstacleAvoidanceCapability") != res.end());

  onto_ptr->feeder.removeInheritage("bumperLeft", "Bumper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ObstacleAvoidanceCapability") != res.end());

  onto_ptr->feeder.removeInheritage("bumperRight", "Bumper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ObstacleAvoidanceCapability") == res.end());
}

TEST(reasoning_anonymous_class, two_equivalences_deletion)
{
  std::vector<std::string> res;

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addRelation("a", "hasComponent", "b");
  onto_ptr->feeder.addInheritage("b", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RGBVisionCapa") != res.end());

  onto_ptr->feeder.addRelation("a", "hasComponent", "realsense");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RGBVisionCapa") != res.end());

  onto_ptr->feeder.removeInheritage("b", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RGBVisionCapa") != res.end());

  onto_ptr->feeder.removeRelation("a", "hasComponent", "realsense");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RGBVisionCapa") == res.end());

  onto_ptr->feeder.addInheritage("b", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RGBVisionCapa") != res.end());
}

TEST(reasoning_anonymous_class, same_as_one_of)
{
  std::vector<std::string> res;

  onto_ptr->feeder.addConcept("the_builder");
  onto_ptr->feeder.addRelation("the_builder", "=", "bob");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Bob") != res.end());

  onto_ptr->feeder.addConcept("the_builder_capa");
  onto_ptr->feeder.addRelation("the_builder_capa", "=", "bob_capa");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder_capa");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "BobInstances") != res.end());

  onto_ptr->feeder.removeRelation("the_builder", "=", "bob");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder_capa");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "BobInstances") != res.end());

  onto_ptr->feeder.removeRelation("the_builder_capa", "=", "bob_capa");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder_capa");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "BobInstances") == res.end());
}

TEST(global_tests, card_value)
{
  std::vector<std::string> res;

  // object property
  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "hasCamera", "realsense");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RealsenseOwner") != res.end());

  onto_ptr->feeder.removeProperty("a", "hasCamera", "realsense");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RealsenseOwner") == res.end());

  onto_ptr->feeder.addConcept("realsense2");
  onto_ptr->feeder.addProperty("realsense", "=", "realsense2");
  onto_ptr->feeder.addProperty("a", "hasCamera", "realsense2");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RealsenseOwner") != res.end());

  onto_ptr->feeder.removeProperty("realsense", "=", "realsense2");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "RealsenseOwner") == res.end());

  // data property
  onto_ptr->feeder.addConcept("b");
  onto_ptr->feeder.addProperty("b", "has_node", "boolean#true");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("b");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Capability") != res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "YoloAlgo") != res.end());

  onto_ptr->feeder.removeProperty("b", "has_node", "boolean#true");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("b");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Capability") == res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "YoloAlgo") == res.end());

  onto_ptr->feeder.addProperty("b", "has_node", "boolean#false");
  onto_ptr->feeder.waitUpdate(1000);
  
  res = onto_ptr->individuals.getUp("b");
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Capability") == res.end());
  EXPECT_TRUE(std::find(res.begin(), res.end(), "YoloAlgo") != res.end());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_reasoning_anonymous_class_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");
  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
