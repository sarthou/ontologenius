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
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  onto_ptr->feeder.waitConnected();
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]cube1|isOn|table1", &callbackAdd, 1);
  EXPECT_NE(id, -1);

  usleep(500000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
    onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  }
  onto_ptr->feeder.waitUpdate(1000);
  usleep(500000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, exact_invert_pattern)
{
  onto_ptr->actions.reset();
  onto_ptr->actions.close();

  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]table1|isUnder|cube1", &callbackAdd, 2);
  EXPECT_NE(id, -1);

  usleep(500000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
    onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  }
  onto_ptr->feeder.waitUpdate(1000);

  usleep(500000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, exact_two_pattern)
{
  onto_ptr->actions.reset();
  onto_ptr->actions.close();

  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]cube1|isOn|table1", &callbackAdd, 1);
  EXPECT_NE(id, -1);
  id = onto_ptr->subscriber.subscribe("[del]cube1|isOn|table1", &callbackDel, 1);
  EXPECT_NE(id, -1);

  usleep(500000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
    onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  }
  onto_ptr->feeder.waitUpdate(500);
  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.removeRelation("cube1", "isOn", "table1");
    onto_ptr->feeder.removeRelation("cube2", "isOn", "table1");
  }
  onto_ptr->feeder.waitUpdate(1000);

  usleep(500000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 1);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, abstract_pattern)
{
  onto_ptr->actions.reset();
  onto_ptr->actions.close();

  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]Cube|isOn|table1", &callbackAdd, 2);
  EXPECT_NE(id, -1);

  usleep(500000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  onto_ptr->feeder.waitUpdate(1000);

  usleep(500000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, variable_pattern)
{
  onto_ptr->actions.reset();
  onto_ptr->actions.close();

  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]?|isOn|table1", &callbackAdd, 3);
  EXPECT_NE(id, -1);

  usleep(500000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table2");
  onto_ptr->feeder.waitUpdate(1000);

  usleep(500000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, variable_abstract_pattern)
{
  onto_ptr->actions.reset();
  onto_ptr->actions.close();

  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]?|isOn|Table", &callbackAdd, 3);
  EXPECT_NE(id, -1);

  usleep(500000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table2");
  onto_ptr->feeder.waitUpdate(1000);

  usleep(500000);

  EXPECT_EQ(done_add, 3);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, any_pattern)
{
  onto_ptr->actions.reset();
  onto_ptr->actions.close();
  done_any = 0;

  auto id = onto_ptr->subscriber.subscribe("[?]cube1|isOn|table1", &callbackAny, 1);
  EXPECT_NE(id, -1);

  usleep(500000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
  onto_ptr->feeder.waitUpdate(500);

  onto_ptr->feeder.removeRelation("cube1", "isOn", "table1");
  onto_ptr->feeder.waitUpdate(1000);

  usleep(500000);

  EXPECT_EQ(done_any, 1);

  onto_ptr->subscriber.cancel(id);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_feature_subscription_test");

  std::thread ros_thread_([]() { std::cout << "SPIN ---------------" << std::endl; ros::spin(); });

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  // std::thread ros_thread_([]() { ros::spin(); });

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
