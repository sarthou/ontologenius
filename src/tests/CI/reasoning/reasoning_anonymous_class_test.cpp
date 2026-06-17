#include <algorithm>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(reasoning_anonymous_class, no_datatype)
{
  std::vector<std::string> res;

  res = onto_ptr->individuals.getUp("garfield", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Cat") != res.end());

  // test if no inference exist at initialization
  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Human") != res.end());

  onto_ptr->feeder.addRelation("alice", "ownPet", "garfield");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 5);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ExclusiveCatOwner") != res.end()); // ownPet only Cat
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})

  onto_ptr->feeder.addRelation("alice", "ownPet", "felix");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 6);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ExclusiveCatOwner") != res.end()); // ownPet only Cat
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PerfectCatOwner") != res.end());   // (PetOwner and (ownPet exactly 2 {felix, garfield, duchesse}))

  onto_ptr->feeder.addRelation("alice", "ownPet", "duchesse");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 6);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ExclusiveCatOwner") != res.end()); // ownPet only Cat
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})
  EXPECT_TRUE(std::find(res.begin(), res.end(), "CrazyCatOwner") != res.end());     // (PetOwner and (ownPet min 3 Cat))

  onto_ptr->feeder.addRelation("alice", "ownPet", "rex");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 4);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})
  EXPECT_TRUE(std::find(res.begin(), res.end(), "CrazyCatOwner") != res.end());     // (PetOwner and (ownPet min 3 Cat))

  onto_ptr->feeder.removeRelation("alice", "ownPet", "duchesse");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 5);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PerfectCatOwner") != res.end());   // (PetOwner and (ownPet exactly 2 {felix, garfield, duchesse}))

  onto_ptr->feeder.removeRelation("alice", "ownPet", "felix");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 4);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))

  onto_ptr->feeder.removeRelation("alice", "ownPet", "garfield");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 4);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "NotCatOwner") != res.end());       // ownPet only (not Cat)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))

  onto_ptr->feeder.removeRelation("alice", "ownPet", "rex");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Human") != res.end());
}

TEST(reasoning_anonymous_class, no_datatype_same_as)
{
  std::vector<std::string> res;

  // test if no inference exist at initialization
  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Human") != res.end());

  onto_ptr->feeder.addRelation("alice", "ownPet", "a");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Animal") != res.end());

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 3);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "NotCatOwner") != res.end());       // ownPet only (not Cat)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))

  onto_ptr->feeder.addRelation("a", "=", "garfield");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Cat") != res.end());

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 5);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "GarfieldOwner") != res.end());     // ownPet value garfield
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ExclusiveCatOwner") != res.end()); // ownPet only Cat
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetOwner") != res.end());          // ownPet some (Cat or {rex, pongo})

  onto_ptr->feeder.removeRelation("a", "=", "garfield");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("a", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Animal") != res.end());

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 3);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "NotCatOwner") != res.end());       // ownPet only (not Cat)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "PetLover") != res.end());          // ((ownPet min 1) and (ownPet max 3))

  onto_ptr->feeder.removeRelation("alice", "ownPet", "a");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("alice", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Human") != res.end());
}

TEST(reasoning_anonymous_class, datatype)
{
  std::vector<std::string> res;

  onto_ptr->feeder.addInheritage("realsense", "Sensor");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Sensor") != res.end());

  onto_ptr->feeder.addRelation("realsense", "isActivated", "boolean#false");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean

  onto_ptr->feeder.addRelation("realsense", "isActivated", "boolean#true");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 2);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true

  onto_ptr->feeder.removeRelation("realsense", "isActivated", "boolean#false");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 3);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean

  onto_ptr->feeder.addRelation("realsense", "hasData", "string#plop");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 5);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean
  EXPECT_TRUE(std::find(res.begin(), res.end(), "StringSensor") != res.end());     // hasData only xsd:string
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorWithData") != res.end());   // hasData some (not errorCode)

  onto_ptr->feeder.addRelation("realsense", "hasData", "string#plip");
  onto_ptr->feeder.addRelation("realsense", "hasData", "string#error");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 7);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean
  EXPECT_TRUE(std::find(res.begin(), res.end(), "StringSensor") != res.end());     // hasData only xsd:string
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorWithData") != res.end());   // hasData some (not errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorInError") != res.end());    // hasData some ({"error", "critical"} or errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "TalkativeSensor") != res.end());  // hasData min 3

  onto_ptr->feeder.addRelation("realsense", "hasData", "errorCode#1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 6);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorWithData") != res.end());   // hasData some (not errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorInError") != res.end());    // hasData some ({"error", "critical"} or errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "TalkativeSensor") != res.end());  // hasData min 3

  onto_ptr->feeder.removeRelation("realsense", "hasData", "string#error");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 6);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorWithData") != res.end());   // hasData some (not errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorInError") != res.end());    // hasData some ({"error", "critical"} or errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "TalkativeSensor") != res.end());  // hasData min 3

  onto_ptr->feeder.removeRelation("realsense", "hasData", "string#plop");
  onto_ptr->feeder.removeRelation("realsense", "hasData", "string#plip");
  onto_ptr->feeder.addRelation("realsense", "hasData", "integer#0");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 5);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorInError") != res.end());    // hasData some ({"error", "critical"} or errorCode)
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorWithData") != res.end());   // hasData some (not errorCode)

  onto_ptr->feeder.removeRelation("realsense", "hasData", "errorCode#1");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 5);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ActiveSensor") != res.end());     // isActivated value true
  EXPECT_TRUE(std::find(res.begin(), res.end(), "ValidSensor") != res.end());      // isActivated exactly 1 xsd:boolean
  EXPECT_TRUE(std::find(res.begin(), res.end(), "BinarySensor") != res.end());     // hasData only {0, 1}
  EXPECT_TRUE(std::find(res.begin(), res.end(), "SensorWithData") != res.end());   // hasData some (not errorCode)

  onto_ptr->feeder.removeRelation("realsense", "hasData", "integer#0");
  onto_ptr->feeder.removeRelation("realsense", "isActivated", "boolean#true");
  onto_ptr->feeder.waitUpdate(1000);

  res = onto_ptr->individuals.getUp("realsense", 1);
  EXPECT_EQ(res.size(), 1);
  EXPECT_TRUE(std::find(res.begin(), res.end(), "Sensor") != res.end());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  onto::OntologyManipulator onto;
  onto_ptr = &onto;

  onto_ptr->reasoners.activate("ontologenius::ReasonerAnonymous");
  onto.close();
  onto.feeder.waitConnected();
  onto.feeder.waitUpdate(1000);

  int res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
