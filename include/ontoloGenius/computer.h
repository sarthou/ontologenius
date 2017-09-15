#include <vector>
#include <string>
#include <set>

#include "ontoloGenius/tree.h"

using namespace std;

struct finder_t
{
  vector<set<string>> words;
  vector<vector<bool>> find;
};

class computer
{
public:
  computer() {};
  ~computer() {};

  bool compute(string equation, tree& onto);

private:
  vector<vector<string>> L;
  vector<vector<string>> R;

  bool split(const string &txt, vector<string> &strs, char ch);
};
