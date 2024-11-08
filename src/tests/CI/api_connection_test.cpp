#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(api_connection, close_call)
{
  EXPECT_TRUE(onto_ptr->close());
}

TEST(api_connection, individuals_call)
{
  std::vector<std::string> res = onto_ptr->individuals.getUp("test");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "ERR:SERVICE_FAIL");
}

TEST(api_connection, objectProperties_call)
{
  std::vector<std::string> res = onto_ptr->objectProperties.getUp("test");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "ERR:SERVICE_FAIL");
}

TEST(api_connection, dataProperties_call)
{
  std::vector<std::string> res = onto_ptr->dataProperties.getUp("test");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "ERR:SERVICE_FAIL");
}

TEST(api_connection, classes_call)
{
  std::vector<std::string> res = onto_ptr->classes.getUp("test");
  EXPECT_NE(res.size(), 1);
  if(res.empty() == false)
    EXPECT_NE(res.front(), "ERR:SERVICE_FAIL");
}

TEST(api_connection, actions_call)
{
  EXPECT_TRUE(onto_ptr->actions.setLang("en"));
}

TEST(api_connection, reasoners_call)
{
  std::vector<std::string> res = onto_ptr->reasoners.list();
  EXPECT_NE(res.size(), 1);
  EXPECT_NE(res.front(), "ERR:SERVICE_FAIL");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_api_connection_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
