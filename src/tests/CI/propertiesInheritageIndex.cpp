#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulatorIndex.h"

onto::OntologyManipulatorIndex* onto_ptr;

TEST(global_tests, class_getRelationFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("human");
  bool res_bool = true;

  res = onto_ptr->classes.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("cube");
  res = onto_ptr->classes.getRelationFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelatedFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->classes.getRelatedFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelationOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  bool res_bool = true;

  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->classes.getRelationOn(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelatedOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->classes.getRelatedOn(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelationWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("human");
  bool res_bool = true;

  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("child");
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("cube");
  res = onto_ptr->classes.getRelationWith(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getRelatedWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  bool res_bool = true;

  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->classes.getRelatedWith(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  int64_t test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  bool res_bool = true;

  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("woman");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("child")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("human")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getFrom(test_word2, test_word);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("man");
  int64_t test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("child");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("woman");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("cube");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->classes.getOn(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, class_getWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("human");
  int64_t test_word2 = onto_ptr->conversion.classesId2Index("man");
  bool res_bool = true;

  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("child");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && (res.size() == 0);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->classes.getWith(test_word, test_word2);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

/*******************
*
* INDIVIDUALS
*
********************/

TEST(global_tests, individual_getRelationFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("alice");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && ((res.size() == 5) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isPositioned")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isOn")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("objectHave3Dposition")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isIn")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("have3Dposition")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->individuals.getRelationFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isOn");
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && ((res.size() == 4) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("cube1")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("mini_box")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("greenCube")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("blueCube")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getRelatedFrom(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelationOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("alice");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasMother")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isUnder")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isPositioned")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getRelationOn(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));

  test_word = onto_ptr->conversion.objectPropertiesId2Index("isIn");
  res = onto_ptr->individuals.getRelatedOn(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("big_box")) != res.end()));

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelationWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("alice");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#2")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  res = onto_ptr->individuals.getRelationWith(test_word);
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#236")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("big_box")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("blueCube")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("redCube")) != res.end()));
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getRelatedWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  bool res_bool = true;

  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 3) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 1) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && ((res.size() == 2) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
                          (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));

  test_word = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getRelatedWith(test_word);
  res_bool = res_bool && (res.size() == 0);

  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getFrom)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.classesId2Index("woman");
  int64_t test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  bool res_bool = true;

  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("woman");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("human");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#0");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("kevin")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.literalsId2Index("integer#2");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->individuals.getFrom(test_word2, test_word);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getOn)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("bob");
  int64_t test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  bool res_bool = true;

  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.literalsId2Index("integer#0")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasFather");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasMother");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("woman")) != res.end()) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.classesId2Index("man")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("kevin");
  test_word2 = onto_ptr->conversion.objectPropertiesId2Index("hasParent");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("alice")) != res.end()) &&
             (find(res.begin(), res.end(), onto_ptr->conversion.individualsId2Index("bob")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.classesId2Index("man");
  test_word2 = onto_ptr->conversion.dataPropertiesId2Index("hasLeg");
  res = onto_ptr->individuals.getOn(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);
}

TEST(global_tests, individual_getWith)
{
  std::vector<int64_t> res;
  int64_t test_word = onto_ptr->conversion.individualsId2Index("kevin");
  int64_t test_word2 = onto_ptr->conversion.individualsId2Index("bob");
  bool res_bool = true;

  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.classesId2Index("man");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasFather")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("hasParent")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#0");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.dataPropertiesId2Index("hasLeg")) != res.end()));
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("alice");
  test_word2 = onto_ptr->conversion.classesId2Index("human");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("bob");
  test_word2 = onto_ptr->conversion.literalsId2Index("integer#2");
  res = onto_ptr->individuals.getWith(test_word, test_word2);
  res_bool = (res.size() == 0);
  EXPECT_TRUE(res_bool);

  test_word = onto_ptr->conversion.individualsId2Index("cube1");
  test_word2 = onto_ptr->conversion.individualsId2Index("redCube");
  res = onto_ptr->individuals.getWith(test_word, test_word2); // use same as (cube1 = greenCube)
  res_bool = ((res.size() == 2) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isPositioned")) != res.end()) &&
              (find(res.begin(), res.end(), onto_ptr->conversion.objectPropertiesId2Index("isOn")) != res.end()));
  EXPECT_TRUE(res_bool);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_global_tester");

  onto::OntologyManipulatorIndex onto;
  onto_ptr = &onto;

  onto.close();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
