#include <chrono>
#include <gtest/gtest.h>
#include <iostream>
#include <ratio>
#include <string>

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/utils/Commands.h"

using namespace std::chrono;

TEST(feature_deep_copy, copy)
{
  ontologenius::Ontology onto1;
  onto1.setDisplay(false);

  const std::string path_base = ontologenius::findPackage("ontologenius");
  std::string path = path_base + "/files/attribute.owl";
  onto1.readFromFile(path);
  path = path_base + "/files/positionProperty.owl";
  onto1.readFromFile(path);
  path = path_base + "/files/testIndividuals.owl";
  onto1.readFromFile(path);
  onto1.close();

  const high_resolution_clock::time_point t1 = high_resolution_clock::now();
  ontologenius::Ontology onto2 = onto1;
  onto2.setDisplay(false);
  const high_resolution_clock::time_point t2 = high_resolution_clock::now();
  const duration<double, std::milli> time_span = duration_cast<duration<double, std::milli>>(t2 - t1);
  std::cout << "copy done in " << time_span.count() << " ms" << std::endl;

  const std::string indiv = "red_cube";

  EXPECT_EQ(onto1.individual_graph_.getUp(indiv).size(), 11);
  EXPECT_EQ(onto2.individual_graph_.getUp(indiv).size(), 11);

  std::string type = "cube";
  onto2.individual_graph_.removeInheritage(indiv, type);
  type = "object";
  onto2.individual_graph_.addInheritage(indiv, type);

  std::cout << "onto1 " << onto1.individual_graph_.getUp(indiv).size() << std::endl;
  std::cout << "onto2 " << onto2.individual_graph_.getUp(indiv).size() << std::endl;

  EXPECT_EQ(onto1.individual_graph_.getUp(indiv).size(), 11);
  EXPECT_EQ(onto2.individual_graph_.getUp(indiv).size(), 7);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
