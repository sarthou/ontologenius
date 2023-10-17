#ifndef ONTOLOGENIUS_SPARQL_H
#define ONTOLOGENIUS_SPARQL_H

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/ontologyOperators/SparqlUtils.h"

#include "ontologenius/graphical/Display.h"

#include <regex>

namespace ontologenius
{

class Sparql
{
public:
  Sparql();
  void link(Ontology* onto) { onto_ = onto; }

  std::pair<std::vector<std::string>, std::vector<std::vector<std::string>>> runStr(const std::string& query, bool single_same = false);
  std::pair<std::vector<std::string>, std::vector<std::vector<index_t>>> runIndex(const std::string& query, bool single_same = false);

  std::string getError() { return error_; }

private:
  ontologenius::Ontology* onto_;
  std::string error_;
  std::regex sparql_pattern_;
  std::map<std::string, SparqlOperator_e> operators_;
  bool single_same_;

  template<typename T> std::pair<std::vector<std::string>, std::vector<std::vector<T>>> run(const std::string& query);

  template<typename T> std::vector<std::vector<T>> resolve(std::vector<triplet_t<T>> query, SparqlOperator_e op, const std::vector<std::vector<T>>& prev_res);
  template<typename T> std::vector<std::vector<T>> resolve(const std::vector<triplet_t<T>>& query, const std::vector<T>& accu);
  template<typename T> void resolveSubQuery(triplet_t<T> triplet, const std::vector<T>& accu, int64_t& var_name, std::unordered_set<T>& values);

  template<typename T> std::unordered_set<T> getOn(const triplet_t<T>& triplet, const T& selector);
  template<typename T> std::unordered_set<T> getFrom(const triplet_t<T>& triplet, const T& selector);
  template<typename T> std::unordered_set<T> getUp(const triplet_t<T>& triplet, const T& selector);
  template<typename T> std::unordered_set<T> getType(const triplet_t<T>& triplet, const T& selector);
  template<typename T> std::unordered_set<T> find(const triplet_t<T>& triplet, const T& selector);
  template<typename T> std::unordered_set<std::string> getName(const triplet_t<T>& triplet, const std::string& selector);

  std::string getPattern(const std::string& text);
  std::vector<SparqlBlock_t> getBlocks(std::string query);
  template<typename T> std::vector<triplet_t<T>> getTriplets(const std::string& query, const std::string& delim);

  template<typename T> void mergeNoOp(const std::vector<T>& base, std::vector<std::vector<T>>& res);
  template<typename T> void mergeNotExists(const std::vector<T>& base, std::vector<std::vector<T>>& res);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQL_H
