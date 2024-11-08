#ifndef ONTOLOGENIUS_SPARQL_H
#define ONTOLOGENIUS_SPARQL_H

#include <regex>

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/ontologyOperators/SparqlUtils.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

  class Sparql
  {
  public:
    Sparql();
    void link(Ontology* onto) { onto_ = onto; }

    std::pair<std::vector<std::string>, std::vector<std::vector<std::string>>> runStr(const std::string& query, bool single_same = false);
    std::pair<std::vector<std::string>, std::vector<std::vector<index_t>>> runIndex(const std::string& query, bool single_same = false);

    std::string getError() const { return error_; }

  private:
    ontologenius::Ontology* onto_;
    std::string error_;
    std::regex sparql_pattern_;
    std::map<std::string, SparqlOperator_e> operators_;
    bool single_same_;

    template<typename T>
    std::pair<std::vector<std::string>, std::vector<std::vector<T>>> run(const std::string& query);

    template<typename T>
    std::vector<std::vector<T>> resolve(std::vector<SparqlTriplet_t<T>> query, SparqlOperator_e op, const std::vector<std::vector<T>>& prev_res);
    template<typename T>
    std::vector<std::vector<T>> resolve(const std::vector<SparqlTriplet_t<T>>& query, const std::vector<T>& accu);
    template<typename T>
    void resolveSubQuery(SparqlTriplet_t<T> triplet, const std::vector<T>& accu, int64_t& var_index, std::unordered_set<T>& values);

    template<typename T>
    std::unordered_set<T> getOn(const SparqlTriplet_t<T>& triplet, const T& selector);
    template<typename T>
    std::unordered_set<T> getFrom(const SparqlTriplet_t<T>& triplet, const T& selector);
    template<typename T>
    std::unordered_set<T> getUp(const SparqlTriplet_t<T>& triplet, const T& selector);
    template<typename T>
    std::unordered_set<T> getType(const SparqlTriplet_t<T>& triplet, const T& selector);
    template<typename T>
    std::unordered_set<T> find(const SparqlTriplet_t<T>& triplet, const T& selector);
    template<typename T>
    std::unordered_set<std::string> getName(const SparqlTriplet_t<T>& triplet, const std::string& selector);

    std::string getPattern(const std::string& text);
    std::vector<SparqlBlock_t> getBlocks(std::string query);
    template<typename T>
    std::vector<SparqlTriplet_t<T>> getTriplets(const std::string& query, const std::string& delim);

    template<typename T>
    void mergeNoOp(const std::vector<T>& base, std::vector<std::vector<T>>& res);
    template<typename T>
    void mergeNotExists(const std::vector<T>& base, std::vector<std::vector<T>>& res);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQL_H
