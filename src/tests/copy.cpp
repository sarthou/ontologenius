#include "ontoloGenius/core/ontoGraphs/Ontology.h"

#include <chrono>

using namespace std::chrono;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_cpy");

  ontologenius::Ontology onto1;
  onto1.readFromUri("https://raw.githubusercontent.com/LAAS-HRI/semantic_route_description/master/files/ideapark/ideapark.owl");
  onto1.readFromUri("https://raw.githubusercontent.com/LAAS-HRI/semantic_route_description/master/files/ideapark/place_description.owl");
  onto1.close();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  ontologenius::Ontology onto2 = onto1;
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double, std::milli> time_span = duration_cast<duration<double, std::milli>>(t2 - t1);
  std::cout << "copy done in " << time_span.count() << " ms" << std::endl;

  std::string indiv = "Burger_King";
  std::string type = "Burger_King_";
  onto2.individual_graph_.removeInheritage(indiv, type);
  type = "shop";
  onto2.individual_graph_.addInheritage(indiv, type);

  std::cout << "onto1 " << onto1.individual_graph_.getUp(indiv).size() << std::endl;
  std::cout << "onto2 " << onto2.individual_graph_.getUp(indiv).size() << std::endl;

  return 0;
}
