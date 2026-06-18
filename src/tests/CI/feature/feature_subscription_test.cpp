#include <atomic>
#include <cstddef>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <unistd.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;
std::atomic<int> done_add;
std::atomic<int> done_del;
std::atomic<int> done_any;

void callbackAdd(const std::string& fact)
{
  (void)fact;
  done_add++;
}

void callbackDel(const std::string& fact)
{
  (void)fact;
  done_del++;
}

void callbackAny(const std::string& fact)
{
  (void)fact;
  done_any++;
}

TEST(feature_subscription, exact_one_pattern)
{
  onto_ptr->feeder.waitConnected();
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]cube1|isOn|table1", &callbackAdd, 1);
  EXPECT_NE(id, -1);

  usleep(100000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addRelation("cube1", "isOn", "table1");
    onto_ptr->feeder.addRelation("cube2", "isOn", "table1");
  }
  bool updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, exact_invert_pattern)
{
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]table2|isUnder|cube1", &callbackAdd, 2);
  EXPECT_NE(id, -1);

  usleep(100000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addRelation("cube1", "isOn", "table2");
    onto_ptr->feeder.addRelation("cube2", "isOn", "table2");
  }
  bool updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, exact_two_pattern)
{
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]cube1|isOn|table3", &callbackAdd, 1);
  EXPECT_NE(id, -1);
  id = onto_ptr->subscriber.subscribe("[del]cube1|isOn|table3", &callbackDel, 1);
  EXPECT_NE(id, -1);

  usleep(100000);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.addRelation("cube1", "isOn", "table3");
    onto_ptr->feeder.addRelation("cube2", "isOn", "table3");
  }
  bool updated = onto_ptr->feeder.waitUpdate(500);
  EXPECT_TRUE(updated);

  for(size_t i = 0; i < 2; i++)
  {
    onto_ptr->feeder.removeRelation("cube1", "isOn", "table3");
    onto_ptr->feeder.removeRelation("cube2", "isOn", "table3");
  }
  updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_add, 1);
  EXPECT_EQ(done_del, 1);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, abstract_pattern)
{
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]Cube|isOn|table4", &callbackAdd, 2);
  EXPECT_NE(id, -1);

  usleep(100000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table4");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table4");
  bool updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, variable_pattern)
{
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]?|isOn|table5", &callbackAdd, 3);
  EXPECT_NE(id, -1);

  usleep(100000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table5");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table5");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table6");
  bool updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, variable_abstract_pattern)
{
  done_add = 0;
  done_del = 0;
  auto id = onto_ptr->subscriber.subscribe("[add]?|isOn|Table", &callbackAdd, 3);
  EXPECT_NE(id, -1);

  usleep(100000);

  onto_ptr->feeder.addInheritage("table7", "Table");
  onto_ptr->feeder.addRelation("cube1", "isOn", "table7");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table7");
  onto_ptr->feeder.addRelation("cube2", "isOn", "table8");
  bool updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_add, 2);
  EXPECT_EQ(done_del, 0);

  onto_ptr->subscriber.cancel(id);
}

TEST(feature_subscription, any_pattern)
{
  done_any = 0;

  auto id = onto_ptr->subscriber.subscribe("[?]cube1|isOn|table9", &callbackAny, 1);
  EXPECT_NE(id, -1);

  usleep(100000);

  onto_ptr->feeder.addRelation("cube1", "isOn", "table9");
  onto_ptr->feeder.removeRelation("cube1", "isOn", "table9");
  bool updated = onto_ptr->feeder.waitUpdate(1000);
  EXPECT_TRUE(updated);

  usleep(500000);

  EXPECT_EQ(done_any, 1);

  onto_ptr->subscriber.cancel(id);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto.close();

  int res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
