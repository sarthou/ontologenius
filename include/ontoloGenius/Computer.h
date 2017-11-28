#include <vector>
#include <string>
#include <set>
#include <iostream>

#include "ontoloGenius/TreeObject.h"

using namespace std;

struct finder_t
{
  vector<set<string>> words;
  vector<vector<bool>> find;
};

class Computer
{
public:
  Computer() {}
  ~Computer() {}

  bool compute(string equation, TreeObject& onto);

private:
  vector<vector<string>> L;
  vector<vector<bool>> notL;
  vector<vector<string>> R;
  vector<vector<bool>> notR;

  bool split(const string &txt, vector<string> &strs, char ch);
  bool split(const string &txt, vector<string> &strs, string delim);
};
