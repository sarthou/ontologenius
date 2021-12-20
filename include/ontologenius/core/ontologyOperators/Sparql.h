#ifndef ONTOLOGENIUS_SPARQL_H
#define ONTOLOGENIUS_SPARQL_H

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/ontologyOperators/SparqlUtils.h"

#include <regex>

namespace ontologenius
{

enum SparqlOperator_e
{
  sparql_none,
  sparql_not_exists
};

struct SparqlBlock_t
{
  std::string raw;
  SparqlOperator_e op;
  std::vector<std::map<std::string, std::string>> res;
  std::vector<SparqlBlock_t> sub_blocks;
};

class Sparql
{
public:
  Sparql();
  void link(Ontology* onto) { onto_ = onto; }

  std::vector<std::map<std::string, std::string>> run(const std::string& query);

  std::string getError() { return error_; }

private:
  ontologenius::Ontology* onto_;
  std::string error_;
  std::regex sparql_pattern_;
  std::map<std::string, SparqlOperator_e> operators_;

  std::vector<std::map<std::string, std::string>> resolve(std::vector<triplet_t> query, SparqlOperator_e op = sparql_none, const std::vector<std::map<std::string, std::string>>& prev_res = {});
  std::vector<std::map<std::string, std::string>> resolve(const std::vector<triplet_t>& query, const std::map<std::string, std::string>& accu);
  void resolveSubQuery(triplet_t triplet, const std::map<std::string, std::string>& accu, std::string& var_name, std::unordered_set<std::string>& values);

  std::unordered_set<std::string> getOn(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getFrom(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getUp(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getType(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> find(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getName(const triplet_t& triplet, const std::string& selector = "");

  std::string getPattern(const std::string& text);
  std::vector<SparqlBlock_t> getBlocks(std::string query);
  std::vector<triplet_t> getTriplets(const std::string& query, const std::string& delim);

  void mergeNoOp(const std::map<std::string, std::string>& base, std::vector<std::map<std::string, std::string>>& res);
  void mergeNotExists(const std::map<std::string, std::string>& base, std::vector<std::map<std::string, std::string>>& res);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQL_H
