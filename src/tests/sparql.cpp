#include <vector>
#include <string>
#include <map>
#include <thread>

#include "ontologenius/core/ontologyOperators/SparqlSolver.h"
#include "ontologenius/core/ontologyOperators/Sparql.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/RosInterface.h"

#include "time.h"
#include <chrono>

using namespace std::chrono;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_sparql");

  ros::NodeHandle nh;
  ontologenius::Ontology* onto = nullptr;
  ontologenius::RosInterface interface;
  std::thread onto_thread;

  std::vector<std::string> files = {
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/exp_director_task/dt_resources/ontologies/common_ground.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/exp_director_task/dt_resources/ontologies/dt_objects.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/exp_director_task/dt_resources/ontologies/dt_setup_1.owl"
  };
  std::string config = "none";

  std::vector<std::string> fils_taboo = {
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/reg/resources/ontology/tests/linkdmdb/linkdmdb_upper.owl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/reg/resources/ontology/tests/linkdmdb/linkdmdb.ttl",
    "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/reg/resources/ontology/tests/linkdmdb/foaf.owl"
  };
  std::string config_taboo = "/home/gsarthou/Robots/Pr2/Semantic/catkin_ws/src/referring_expression_generation/reg/resources/conf/no_generalization.yaml";

  interface.setDisplay(false);
  interface.init("en", "none", fils_taboo, config_taboo);
  onto = interface.getOntology();
  onto_thread = std::thread(&ontologenius::RosInterface::run, &interface);
  interface.close();

  //std::string query = "SELECT * WHERE {?0 isA Container. ?0 hasIn ?1. ?0 isOnTopOf ?2. ?1 isA Cube. ?1 hasFigure ?3. ?3 isA Circle}";
  std::string query = "SELECT * WHERE {?0 isA movie_director. ?0 movie_performance ?2. ?2 isA movie_performance}";
  ontologenius::SparqlSolver sparql;
  sparql.link(onto);
  sparql.set(query);

  std::vector<std::map<std::string, std::string>> solutions;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  for(ontologenius::SparqlSolver::iterator it = sparql.begin(); it != sparql.end(); ++it)
  {
    solutions.push_back(*it);
    /*auto binding = *it;
    std::cout << "--------" << std::endl;
    for(auto& b : binding)
        std::cout << b.first << " : " << b.second << std::endl;*/
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "SparqlSolver has found " << solutions.size() << " solutions in " << time_span.count()*1000 << "ms" << std::endl;

  ontologenius::Sparql sparql_base;
  sparql_base.link(onto);
  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  auto sol_base = sparql_base.run(query);
  high_resolution_clock::time_point t4 = high_resolution_clock::now();
  time_span = duration_cast<duration<double>>(t4 - t3);

  std::cout << "Sparql has found " << sol_base.size() << " solutions in " << time_span.count()*1000 << "ms" << std::endl;

  return 0;
}