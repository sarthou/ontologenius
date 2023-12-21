#include <vector>
#include <string>
#include <map>
#include <thread>

#include "ontologenius/core/ontologyOperators/SparqlSolver.h"
#include "ontologenius/core/ontologyOperators/Sparql.h"
#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/interface/RosInterface.h"

#include "time.h"
#include <chrono>

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std::chrono;

void handler(int sig)
{
  void *array[10];
  size_t size;

  size = backtrace(array, 10);

  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

template <typename Map>
bool mapCompare (Map const &lhs, Map const &rhs)
{
  // No predicate needed because there is operator== for pairs already.
  return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

void compareSolutions(const std::vector<std::map<std::string, std::string>>& sol1, const std::vector<std::map<std::string, std::string>>& sol2)
{
  if(sol1.size() != sol2.size())
  {
    std::cout << "[ERROR] not the same number of solutions. " << sol1.size() << " vs " << sol2.size() << std::endl;
    return;
  }

  size_t nb_diff = 0;
  for(auto& s1 : sol1)
  {
    bool found = false;
    for(auto& s2 : sol2)
    {
      if(mapCompare(s1, s2))
      {
        found = true;
        break;
      }
    }

    if(found == false)
      nb_diff++;
  }

  if(nb_diff)
    std::cout << "[ERROR] " << nb_diff << " solutions where not identicals" << std::endl;
  else
    std::cout << "[SUCCESS]" << std::endl;
}

std::map<std::string, std::string> runSparqlSolverFirst(ontologenius::SparqlSolver& solver, const std::string& query)
{
  solver.set(query);

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  std::map<std::string, std::string> solutions = *solver.begin();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "SparqlSolver has found first solution in " << time_span.count()*1000 << "ms" << std::endl;

  return solutions;
}

std::vector<std::map<std::string, std::string>> runSparqlSolver(ontologenius::SparqlSolver& solver, const std::string& query)
{
  solver.set(query);

  std::vector<std::map<std::string, std::string>> solutions;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  for(ontologenius::SparqlSolver::iterator it = solver.begin(); it != solver.end(); ++it)
  {
    solutions.push_back(*it);
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "SparqlSolver has found " << solutions.size() << " solutions in " << time_span.count()*1000 << "ms" << std::endl;

  return solutions;
}

std::vector<std::vector<std::string>> runSparqlBase(ontologenius::Sparql& solver, const std::string& query)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  auto solutions = solver.runStr(query);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Sparql has found " << solutions.second.size() << " solutions in " << time_span.count()*1000 << "ms" << std::endl;

  return solutions.second;
}

std::vector<std::vector<ontologenius::index_t>> runSparqlBaseIndex(ontologenius::Sparql& solver, const std::string& query)
{
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  auto solutions = solver.runIndex(query);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Sparql index has found " << solutions.second.size() << " solutions in " << time_span.count()*1000 << "ms" << std::endl;

  return solutions.second;
}

void configureInterface(ontologenius::RosInterface& interface)
{
  std::vector<std::string> files = {
    "/home/gsarthou/Robots/Pr2/Dacobot/catkin_ws/src/dt_resources/ontologies/dt_setup_1.owl"
  };

  interface.setDisplay(false);
  interface.init("en", "none", files, "none");
}

void configureInterfaceTaboo(ontologenius::RosInterface& interface)
{
  std::vector<std::string> files = {
    "/home/gsarthou/Robots/Pr2/Dacobot/catkin_ws/src/referring_expression_generation/resources/ontology/tests/linkdmdb/linkdmdb_upper.owl",
    "/home/gsarthou/Robots/Pr2/Dacobot/catkin_ws/src/referring_expression_generation/resources/ontology/tests/linkdmdb/linkdmdb.ttl",
    "/home/gsarthou/Robots/Pr2/Dacobot/catkin_ws/src/referring_expression_generation/resources/ontology/tests/linkdmdb/foaf.owl"
  };
  std::string config = "/home/gsarthou/Robots/Pr2/Dacobot/catkin_ws/src/referring_expression_generation/resources/conf/no_generalization.yaml";

  interface.setDisplay(false);
  interface.init("en", "none", files, config);
}

void testInsertIndex(size_t nb)
{
  std::unordered_set<ontologenius::index_t> set;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(size_t i = 0; i < nb; i++)
  {
    set.insert(i);
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "Insertion of " << nb << " long in " << time_span.count()*1000 << "ms" << std::endl;
}

int main(int argc, char** argv)
{
  signal(SIGSEGV, handler);
  ros::init(argc, argv, "ontologenius_sparql");

  ontologenius::Ontology* onto = nullptr;
  ontologenius::RosInterface interface;
  std::thread onto_thread;

  configureInterfaceTaboo(interface);
  onto = interface.getOntology();
  onto_thread = std::thread(&ontologenius::RosInterface::run, &interface);
  interface.close();

  //std::string query = "SELECT * WHERE {?0 isA Container. ?0 hasIn ?1. ?0 isOnTopOf ?2. ?1 isA Cube. ?1 hasFigure ?3. ?3 isA Circle}";
  //std::string query = "SELECT * WHERE {?0 isA movie_director. ?0 movie_performance ?2. ?2 isA movie_performance}";
  //std::string query = "?0 isA movie_director, ?0 movie_performance ?8, ?8 isA movie_performance, ?8 performance_actor string#Dany Boon";
  std::string query = "SELECT * WHERE {?0 isA movie_actor}";

  ontologenius::SparqlSolver sparql;
  sparql.link(onto);
  runSparqlSolverFirst(sparql, query);
  auto solution_it = runSparqlSolver(sparql, query);

  ontologenius::Sparql sparql_base;
  sparql_base.link(onto);

  for(size_t i = 0; i < 3; i++)
  {
    query = "SELECT * WHERE {?0 isA movie_actor}";
    auto solution_base = runSparqlBase(sparql_base, query);

    usleep(1000000);

    query = "SELECT * WHERE {?0 isA " + std::to_string(onto->class_graph_.getIndex("movie_actor")) + "}";
    auto solution_index = runSparqlBaseIndex(sparql_base, query);

    usleep(1000000);

    query = "SELECT * WHERE {?0 isA movie_director. ?0 movie_performance ?2. ?2 isA movie_performance}";
    solution_base = runSparqlBase(sparql_base, query);

    usleep(1000000);

    query = "SELECT * WHERE {?0 isA " + std::to_string(onto->class_graph_.getIndex("movie_director")) + 
            ". ?0 " + std::to_string(onto->class_graph_.getIndex("movie_performance")) + 
            " ?2. ?2 isA " + std::to_string(onto->class_graph_.getIndex("movie_performance")) + 
            "}";
    solution_index = runSparqlBaseIndex(sparql_base, query);
  }

  interface.stop();
  onto_thread.join();

  return 0;
}