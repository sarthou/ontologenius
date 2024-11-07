#include <algorithm>
#include <cstddef>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;
std::atomic<int> done_add;
std::atomic<int> done_del;
std::atomic<int> done_any;

#define NB_TIME 20 // 2s

void callbackAdd(const std::string& fact)
{
  done_add++;
}

void callbackDel(const std::string& fact)
{
  done_del++;
}

void callbackAny(const std::string& fact)
{
  done_any++;
}

TEST(feature_subscription, exact_one_pattern)
{
  done_add = 0;
  done_del = 0;
  onto_ptr->subscriber.subscribe("[add]cube1|isOn|table1", &callbackAdd, 1);

  usleep(1000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addProperty("cube1", "isOn", "table1");
    onto_ptr->feeder.addProperty("cube2", "isOn", "table1");
  }

  usleep(1000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 0);
}

TEST(feature_subscription, exact_invert_pattern)
{
  done_add = 0;
  done_del = 0;
  onto_ptr->subscriber.subscribe("[add]table1|isUnder|cube1", &callbackAdd, 2);

  usleep(1000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addProperty("cube1", "isOn", "table1");
    onto_ptr->feeder.addProperty("cube2", "isOn", "table1");
  }

  usleep(1000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);
}

TEST(feature_subscription, exact_two_pattern)
{
  done_add = 0;
  done_del = 0;
  onto_ptr->subscriber.subscribe("[add]cube1|isOn|table1", &callbackAdd, 1);
  onto_ptr->subscriber.subscribe("[del]cube1|isOn|table1", &callbackDel, 1);

  usleep(1000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addProperty("cube1", "isOn", "table1");
    onto_ptr->feeder.addProperty("cube2", "isOn", "table1");
  }
  onto_ptr->feeder.waitUpdate(500);
  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.removeProperty("cube1", "isOn", "table1");
    onto_ptr->feeder.removeProperty("cube2", "isOn", "table1");
  }

  usleep(1000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 1);
}

TEST(feature_subscription, abstract_pattern)
{
  done_add = 0;
  done_del = 0;
  onto_ptr->subscriber.subscribe("[add]Cube|isOn|table1", &callbackAdd, 2);

  usleep(1000);

  onto_ptr->feeder.addProperty("cube1", "isOn", "table1");
  onto_ptr->feeder.addProperty("cube2", "isOn", "table1");

  usleep(1000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);
}

TEST(feature_subscription, variable_pattern)
{
  done_add = 0;
  done_del = 0;
  onto_ptr->subscriber.subscribe("[add]?|isOn|table1", &callbackAdd, 3);

  usleep(1000);

  onto_ptr->feeder.addProperty("cube1", "isOn", "table1");
  onto_ptr->feeder.addProperty("cube2", "isOn", "table1");
  onto_ptr->feeder.addProperty("cube2", "isOn", "table2");

  usleep(1000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);
}

TEST(feature_subscription, variable_abstract_pattern)
{
  done_add = 0;
  done_del = 0;
  onto_ptr->subscriber.subscribe("[add]?|isOn|Table", &callbackAdd, 3);

  usleep(1000);

  onto_ptr->feeder.addProperty("cube1", "isOn", "table1");
  onto_ptr->feeder.addProperty("cube2", "isOn", "table1");
  onto_ptr->feeder.addProperty("cube2", "isOn", "table2");

  usleep(1000);

  EXPECT_EQ(done_add, 3);
  EXPECT_EQ(done_del, 0);
}

TEST(feature_subscription, any_pattern)
{
  done_any = 0;

  onto_ptr->subscriber.subscribe("[?]cube1|isOn|table1", &callbackAny, 1);

  usleep(1000);

  onto_ptr->feeder.addProperty("cube1", "isOn", "table1");

  onto_ptr->feeder.waitUpdate(500);

  onto_ptr->feeder.removeProperty("cube1", "isOn", "table1");

  usleep(1000);

  EXPECT_EQ(done_any, 1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_feature_subscription_test");

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
