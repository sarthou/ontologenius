#ifndef ONTOLOGENIUS_SPARQL_H
#define ONTOLOGENIUS_SPARQL_H

#include "ontologenius/core/ontoGraphs/Ontology.h"

#include <regex>

namespace ontologenius
{

struct resource_t
{
  std::string name;
  bool variable;
  bool regex;
};

struct triplet_t
{
  resource_t subject;
  resource_t predicat;
  resource_t object;
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

  std::vector<std::map<std::string, std::string>> resolve(std::vector<triplet_t> query, const std::map<std::string, std::string>& accu = {});
  void resolveSubQuery(triplet_t triplet, const std::map<std::string, std::string>& accu, std::string& var_name, std::unordered_set<std::string>& values);

  std::unordered_set<std::string> getOn(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getFrom(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getUp(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getType(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> find(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getName(const triplet_t& triplet, const std::string& selector = "");

  std::vector<triplet_t> getTriplets(const std::string& query, const std::string& delim);
  triplet_t getTriplet(const std::string& subquery);
  resource_t getResource(const std::string& resource);
  std::string toString(const triplet_t& triplet);

  void removeUselessSpace(std::string& text);
  void removeChar(std::string& text, const std::vector<char>& delim);

  void filter(std::vector<std::map<std::string, std::string>>& res, const std::vector<std::string>& vars, bool distinct);
  void removeDuplicate(std::vector<std::map<std::string, std::string>>& vect);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQL_H
