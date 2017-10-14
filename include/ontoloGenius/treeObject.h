#include <string>
#include <vector>
#include <set>
#include <stdint.h>

#ifndef TREEOBJECT_H
#define TREEOBJECT_H

using namespace std;

struct branch_t
{
  string value;
  vector<branch_t*> childs;
  vector<branch_t*> mothers;
  vector<branch_t*> disjoints;
  uint8_t family;
  uint8_t nb_mothers;

  branch_t(string p_value) : family(0), nb_mothers(0)
    {value = p_value; };
};

class tree_drawer;
class treeProperty;

class treeObject
{
  friend tree_drawer;
  friend treeProperty;
public:
  treeObject() {}
  ~treeObject();

  void add(string value, vector<string>& mothers, vector<string>& disjoints);
  void add(vector<string>& disjoints);
  void close();

  set<string> getDown(string value);
  set<string> getUp(string value);
  set<string> getDisjoint(string value);

private:
  vector<branch_t*> branchs;
  vector<branch_t*> roots;

  vector<branch_t*> tmp_mothers;

  int depth;

  void link();
  void add_family(branch_t* branch, uint8_t family);

  set<string> getDown(branch_t* branch, string value);
  set<string> getUp(branch_t* branch, string value);
};

#endif /* TREEOBJECT_H */
