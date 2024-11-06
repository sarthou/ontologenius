#include <algorithm>
#include <cstdint>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <vector>

#include "ontologenius/API/ontologenius/OntologyManipulatorIndex.h"

onto::OntologyManipulatorIndex* onto_ptr;

TEST(api_relations_index, class_getRelationFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("human");
  bool res_bool = true;

  res = onto_ptr->classes.getRelationFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getRelationFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("cube");
  res = onto_ptr->classes.getRelationFrom(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getRelatedFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getRelatedFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getRelatedFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->classes.getRelatedFrom(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getRelationOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  bool res_bool = true;

  res = onto_ptr->classes.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->classes.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->classes.getRelationOn(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getRelatedOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->classes.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->classes.getRelatedOn(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getRelationWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("human");
  bool res_bool = true;

  res = onto_ptr->classes.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("child");
  res = onto_ptr->classes.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("cube");
  res = onto_ptr->classes.getRelationWith(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getRelatedWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->classes.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->classes.getRelatedWith(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  int64_t test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  bool res_bool = true;

  res = onto_ptr->classes.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("woman");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("man");
  int64_t test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->classes.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("child");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("woman");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("cube");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, class_getWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("human");
  int64_t test_word2 = onto_ptr->conversion.classesId2Index("man");
  bool res_bool = true;

  res = onto_ptr->classes.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("child");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  EXPECT_TRUE(res.empty());

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  EXPECT_TRUE(res.empty());
}

/*******************
 *
 * INDIVIDUALS
 *
 ********************/

TEST(api_relations_index, individual_getRelationFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("alice");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  res = onto_ptr->individuals.getRelationFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  res = onto_ptr->individuals.getRelationFrom(test_word);
  EXPECT_EQ(res.size(), 5);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isPositioned")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isOn")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("objectHave3Dposition")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isIn")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("have3Dposition")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->individuals.getRelationFrom(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, individual_getRelatedFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedFrom(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("cube1")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("mini_box")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("green_cube")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("blue_cube")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, individual_getRelationOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("alice");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  res = onto_ptr->individuals.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isUnder")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isPositioned")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->individuals.getRelationOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getRelationOn(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, individual_getRelatedOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isIn");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("big_box")) != res.end());
  EXPECT_TRUE(res_bool);
}

TEST(api_relations_index, individual_getRelationWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("alice");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  res = onto_ptr->individuals.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  res = onto_ptr->individuals.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  res = onto_ptr->individuals.getRelationWith(test_word);
  EXPECT_EQ(res.size(), 4);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#236")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("big_box")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("blue_cube")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("red_cube")) != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(api_relations_index, individual_getRelatedWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, individual_getFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  int64_t test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  bool res_bool = true;

  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("woman");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 3);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, individual_getOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("bob");
  int64_t test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  EXPECT_TRUE(res.empty());
}

TEST(api_relations_index, individual_getWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("kevin");
  int64_t test_word2 = onto_ptr->conversion.individualsId2Index("bob");
  bool res_bool = true;

  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_EQ(res.size(), 1);
  res_bool = (std::find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end());
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_TRUE(res.empty());

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  EXPECT_TRUE(res.empty());

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  test_word2 = onto_ptr->conversion.individualsId2Index("red_cube");
  res = onto_ptr->individuals.getWith(test_word, test_word2); // use same as (cube1 = green_cube)
  EXPECT_EQ(res.size(), 2);
  res_bool = ((std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isPositioned")) != res.end()) &&
              (std::find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isOn")) != res.end()));
  EXPECT_TRUE(res_bool);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_api_relations_index_test");

  onto::OntologyManipulatorIndex onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
