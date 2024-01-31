#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(global_tests, same_as_range_restriction)
{
  std::vector<std::string> res;
  bool res_bool = false;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");

  onto_ptr->feeder.addConcept("indiv2");
  onto_ptr->feeder.addProperty("indiv2", "hasComponent", "realsense_pepper");

  onto_ptr->feeder.addProperty("realsense_d435i", "=", "realsense_pepper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv2");
  
  res_bool = find(res.begin(), res.end(), "RealSenseVisionCapability") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addProperty("indiv2", "=", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv1");
  res_bool = (find(res.begin(), res.end(), "RealSenseVisionCapability") != res.end());
  EXPECT_TRUE(res_bool); 

  onto_ptr->feeder.addConcept("indiv3");
  onto_ptr->feeder.addProperty("indiv3", "hasCapability", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  res_bool = (find(res.begin(), res.end(), "PepperVisionCapability") != res.end());
  EXPECT_TRUE(res_bool); //

  onto_ptr->feeder.removeProperty("indiv3", "hasCapability", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  res_bool = (find(res.begin(), res.end(), "PepperVisionCapability") == res.end());
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addProperty("indiv3", "hasCapability", "indiv1");
  onto_ptr->feeder.removeProperty("indiv2", "=", "indiv1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  res_bool = (find(res.begin(), res.end(), "PepperVisionCapability") == res.end());
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addProperty("indiv2", "=", "indiv1");
  onto_ptr->feeder.removeProperty("realsense_d435i", "=", "realsense_pepper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("indiv3");
  res_bool = (find(res.begin(), res.end(), "PepperVisionCapability") == res.end());
  EXPECT_TRUE(res_bool);

}

TEST(global_tests, trace_cleaning)
{
  std::vector<std::string> res;
  bool res_bool = false;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "hasComponent", "b");
  onto_ptr->feeder.addProperty("b", "hasCamera", "c");
  onto_ptr->feeder.addInheritage("c", "Camera");
  onto_ptr->feeder.addProperty("b", "hasComponent", "d");
  onto_ptr->feeder.addInheritage("d", "Lidar");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  
  res_bool = find(res.begin(), res.end(), "LocalizeCapability") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeProperty("b", "hasCamera", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "LocalizeCapability") == res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addProperty("b", "hasCamera", "e");
  onto_ptr->feeder.addInheritage("e", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "LocalizeCapability") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeInheritage("c", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = (find(res.begin(), res.end(), "LocalizeCapability") != res.end());
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addInheritage("c", "Camera");
  onto_ptr->feeder.removeInheritage("e", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "LocalizeCapability") == res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addProperty("e", "=", "c");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "LocalizeCapability") != res.end();
  EXPECT_TRUE(res_bool);

}

TEST(global_tests, cardinality_min_testing)
{
  std::vector<std::string> res;
  bool res_bool = false;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");

  onto_ptr->feeder.addProperty("pepper", "hasComponent", "bumperLeft");
  onto_ptr->feeder.addProperty("pepper", "hasComponent", "bumperRight");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  res_bool = find(res.begin(), res.end(), "ObstacleAvoidanceCapability") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addConcept("bumperMiddle");
  onto_ptr->feeder.addInheritage("bumperMiddle", "Bumper");
  onto_ptr->feeder.addProperty("pepper", "hasComponent", "bumperMiddle");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  res_bool = find(res.begin(), res.end(), "ObstacleAvoidanceCapability") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeInheritage("bumperLeft", "Bumper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  res_bool = find(res.begin(), res.end(), "ObstacleAvoidanceCapability") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeInheritage("bumperRight", "Bumper");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("pepper");
  res_bool = (find(res.begin(), res.end(), "ObstacleAvoidanceCapability") == res.end());
  EXPECT_TRUE(res_bool);

}

TEST(global_tests, two_equivalences_deletion)
{
  std::vector<std::string> res;
  bool res_bool = false;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");

  onto_ptr->feeder.addConcept("a");
  onto_ptr->feeder.addProperty("a", "hasComponent", "b");
  onto_ptr->feeder.addInheritage("b", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "RGBVisionCapa") != res.end();
  EXPECT_TRUE(res_bool); //

  onto_ptr->feeder.addProperty("a", "hasComponent", "realsense");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "RGBVisionCapa") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeInheritage("b", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "RGBVisionCapa") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeProperty("a", "hasComponent", "realsense");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = (find(res.begin(), res.end(), "RGBVisionCapa") == res.end());
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addInheritage("b", "Camera");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a");
  res_bool = find(res.begin(), res.end(), "RGBVisionCapa") != res.end();
  EXPECT_TRUE(res_bool);

}

TEST(global_tests, same_as_one_of)
{
  std::vector<std::string> res;
  bool res_bool = false;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");

  onto_ptr->feeder.addConcept("the_builder");
  onto_ptr->feeder.addProperty("the_builder", "=", "bob");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder");
  res_bool = find(res.begin(), res.end(), "Bob") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.addConcept("the_builder_capa");
  onto_ptr->feeder.addProperty("the_builder_capa", "=", "bob_capa");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder_capa");
  res_bool = find(res.begin(), res.end(), "BobInstances") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeProperty("the_builder", "=", "bob");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder_capa");
  res_bool = find(res.begin(), res.end(), "BobInstances") != res.end();
  EXPECT_TRUE(res_bool);

  onto_ptr->feeder.removeProperty("the_builder_capa", "=", "bob_capa");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("the_builder_capa");
  res_bool = (find(res.begin(), res.end(), "BobInstances") == res.end());
  EXPECT_TRUE(res_bool);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_anonymous_inference");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}