#ifndef ONTOLOGENIUS_SPARQLUTILS_H
#define ONTOLOGENIUS_SPARQLUTILS_H

#include <string>
#include <vector>
#include <map>

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

triplet_t getTriplet(const std::string& triplet_txt);
resource_t getResource(std::string resource_txt);
std::string toString(const triplet_t& triplet);

void removeUselessSpace(std::string& text);
void removeChar(std::string& text, const std::vector<char>& delim);

void filter(std::vector<std::map<std::string, std::string>>& res, const std::vector<std::string>& vars, bool distinct);
void removeDuplicate(std::vector<std::map<std::string, std::string>>& vect);

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQLUTILS_H