#include <algorithm>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

onto::OntologiesManipulator* onto_ptr;

TEST(feature_multi, create)
{
  std::vector<std::string> res;
  bool res_bool = true;
  std::string test_word = "Robot";

  EXPECT_TRUE(onto_ptr->add("paul"));

  (*onto_ptr)["paul"]->close();

  (*onto_ptr)["paul"]->feeder.waitConnected();
  (*onto_ptr)["paul"]->feeder.addConcept("Human");
  (*onto_ptr)["paul"]->feeder.addInheritage("Man", "Human");
  (*onto_ptr)["paul"]->feeder.addInheritage("Woman", "Human");
  (*onto_ptr)["paul"]->feeder.addInheritage("Human", "Agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("Robot", "Agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("pepper", "Robot");
  (*onto_ptr)["paul"]->feeder.waitUpdate(500);

  res = (*onto_ptr)["paul"]->classes.getUp(test_word);
  res_bool = ((res.size() == 2) &&
              (std::find(res.begin(), res.end(), "Robot") != res.end()) &&
              (std::find(res.begin(), res.end(), "Agent") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(onto_ptr->get("paul") != nullptr);
  EXPECT_TRUE(onto_ptr->del("paul"));
  EXPECT_TRUE(onto_ptr->get("paul") == nullptr);

  EXPECT_TRUE(onto_ptr->get("bob") == nullptr);
  EXPECT_TRUE(onto_ptr->del("bob"));
}

TEST(feature_multi, differences)
{
  std::vector<std::string> res;
  bool res_bool = true;

  EXPECT_TRUE(onto_ptr->add("paul"));
  EXPECT_TRUE(onto_ptr->add("bob"));

  (*onto_ptr)["paul"]->close();
  (*onto_ptr)["bob"]->close();

  (*onto_ptr)["paul"]->feeder.waitConnected();
  (*onto_ptr)["paul"]->feeder.addConcept("Human");
  (*onto_ptr)["paul"]->feeder.addInheritage("Man", "Human");
  (*onto_ptr)["paul"]->feeder.addInheritage("Woman", "Human");
  (*onto_ptr)["paul"]->feeder.addInheritage("Human", "Agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("Robot", "Agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("pepper", "Robot");
  (*onto_ptr)["paul"]->feeder.waitUpdate(500);

  (*onto_ptr)["bob"]->feeder.waitConnected();
  (*onto_ptr)["bob"]->feeder.addConcept("Human");
  (*onto_ptr)["bob"]->feeder.addInheritage("Man", "Human");
  (*onto_ptr)["bob"]->feeder.addInheritage("Woman", "Human");
  (*onto_ptr)["bob"]->feeder.addInheritage("Human", "Agent");
  (*onto_ptr)["bob"]->feeder.addInheritage("Robot", "Agent");
  (*onto_ptr)["bob"]->feeder.addInheritage("pepper", "Human");
  (*onto_ptr)["bob"]->feeder.waitUpdate(500);

  res = onto_ptr->getDifference("paul", "bob", "pepper");
  res_bool = ((res.size() == 2) &&
              (std::find(res.begin(), res.end(), "[+]pepper|isA|Robot") != res.end()) &&
              (std::find(res.begin(), res.end(), "[-]pepper|isA|Human") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->getDifference("bob", "paul", "pepper");
  res_bool = ((res.size() == 2) &&
              (std::find(res.begin(), res.end(), "[-]pepper|isA|Robot") != res.end()) &&
              (std::find(res.begin(), res.end(), "[+]pepper|isA|Human") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(onto_ptr->get("paul") != nullptr);
  EXPECT_TRUE(onto_ptr->del("paul"));
  EXPECT_TRUE(onto_ptr->get("paul") == nullptr);

  EXPECT_TRUE(onto_ptr->get("bob") != nullptr);
  EXPECT_TRUE(onto_ptr->del("bob"));
  EXPECT_TRUE(onto_ptr->get("bob") == nullptr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_feature_multi_test");

  onto::OntologiesManipulator onto;
  onto_ptr = &onto;

  onto.waitInit();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
