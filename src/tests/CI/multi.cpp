#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologiesManipulator.h"

onto::OntologiesManipulator* onto_ptr;

TEST(multi_tests, create)
{
  std::vector<std::string> res;
  bool res_bool = true;
  std::string test_word = "robot";

  EXPECT_TRUE(onto_ptr->add("paul"));

  (*onto_ptr)["paul"]->close();

  (*onto_ptr)["paul"]->feeder.waitConnected();
  (*onto_ptr)["paul"]->feeder.addConcept("human");
  (*onto_ptr)["paul"]->feeder.addInheritage("man", "human");
  (*onto_ptr)["paul"]->feeder.addInheritage("woman", "human");
  (*onto_ptr)["paul"]->feeder.addInheritage("human", "agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("robot", "agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("pepper", "robot");
  (*onto_ptr)["paul"]->feeder.waitUpdate(500);

  res = (*onto_ptr)["paul"]->classes.getUp(test_word);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "robot") != res.end()) &&
              (find(res.begin(), res.end(), "agent") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(onto_ptr->get("paul") != nullptr);
  EXPECT_TRUE(onto_ptr->del("paul"));
  EXPECT_TRUE(onto_ptr->get("paul") == nullptr);

  EXPECT_TRUE(onto_ptr->get("bob") == nullptr);
  EXPECT_TRUE(onto_ptr->del("bob"));
}

TEST(multi_tests, differences)
{
  std::vector<std::string> res;
  bool res_bool = true;

  EXPECT_TRUE(onto_ptr->add("paul"));
  EXPECT_TRUE(onto_ptr->add("bob"));

  (*onto_ptr)["paul"]->close();
  (*onto_ptr)["bob"]->close();

  (*onto_ptr)["paul"]->feeder.waitConnected();
  (*onto_ptr)["paul"]->feeder.addConcept("human");
  (*onto_ptr)["paul"]->feeder.addInheritage("man", "human");
  (*onto_ptr)["paul"]->feeder.addInheritage("woman", "human");
  (*onto_ptr)["paul"]->feeder.addInheritage("human", "agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("robot", "agent");
  (*onto_ptr)["paul"]->feeder.addInheritage("pepper", "robot");
  (*onto_ptr)["paul"]->feeder.waitUpdate(500);

  (*onto_ptr)["bob"]->feeder.waitConnected();
  (*onto_ptr)["bob"]->feeder.addConcept("human");
  (*onto_ptr)["bob"]->feeder.addInheritage("man", "human");
  (*onto_ptr)["bob"]->feeder.addInheritage("woman", "human");
  (*onto_ptr)["bob"]->feeder.addInheritage("human", "agent");
  (*onto_ptr)["bob"]->feeder.addInheritage("robot", "agent");
  (*onto_ptr)["bob"]->feeder.addInheritage("pepper", "human");
  (*onto_ptr)["bob"]->feeder.waitUpdate(500);

  res = onto_ptr->getDifference("paul", "bob", "pepper");
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "[+]pepper|isA|robot") != res.end()) &&
              (find(res.begin(), res.end(), "[-]pepper|isA|human") != res.end()));
  EXPECT_TRUE(res_bool);

  res = onto_ptr->getDifference("bob", "paul", "pepper");
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), "[-]pepper|isA|robot") != res.end()) &&
              (find(res.begin(), res.end(), "[+]pepper|isA|human") != res.end()));
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
  ros::init(argc, argv, "ontologenius_multi_tester");

  onto::OntologiesManipulator onto;
  onto_ptr = &onto;

  onto.waitInit();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
