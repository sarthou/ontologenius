#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/utils/Commands.h"

#include <chrono>

#include <gtest/gtest.h>

using namespace std::chrono;

TEST(cpy_tests, reset)
{
  ontologenius::Ontology onto1;
  onto1.setDisplay(false);

  std::string path_base = ontologenius::findPackage("ontologenius");
  std::string path = path_base + "/files/attribute.owl";
  onto1.readFromFile(path);
  path = path_base + "/files/positionProperty.owl";
  onto1.readFromFile(path);
  path = path_base + "/files/testIndividuals.owl";
  onto1.readFromFile(path);
  onto1.close();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  ontologenius::Ontology onto2 = onto1;
  onto2.setDisplay(false);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double, std::milli> time_span = duration_cast<duration<double, std::milli>>(t2 - t1);
  std::cout << "copy done in " << time_span.count() << " ms" << std::endl;

  std::string indiv = "redCube";

  EXPECT_TRUE(onto1.individual_graph_.getUp(indiv).size() == 11);
  EXPECT_TRUE(onto2.individual_graph_.getUp(indiv).size() == 11);

  std::string type = "cube";
  onto2.individual_graph_.removeInheritage(indiv, type);
  type = "object";
  onto2.individual_graph_.addInheritage(indiv, type);

  std::cout << "onto1 " << onto1.individual_graph_.getUp(indiv).size() << std::endl;
  std::cout << "onto2 " << onto2.individual_graph_.getUp(indiv).size() << std::endl;

  EXPECT_TRUE(onto1.individual_graph_.getUp(indiv).size() == 11);
  EXPECT_TRUE(onto2.individual_graph_.getUp(indiv).size() == 7);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
