#include <vector>
#include <string>
#include <set>

#include "ontoloGenius/treeObject.h"

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

  bool compute(string equation, treeObject& onto);

private:
  vector<vector<string>> L;
  vector<vector<bool>> notL;
  vector<vector<string>> R;
  vector<vector<bool>> notR;

  bool split(const string &txt, vector<string> &strs, char ch);
  bool split(const string &txt, vector<string> &strs, string delim);
};
