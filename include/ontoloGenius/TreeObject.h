#include <string>
#include <vector>
#include <set>
#include <stdint.h>

#ifndef TREEOBJECT_H
#define TREEOBJECT_H

using namespace std;

struct Branch_t
{
  string value_;
  vector<Branch_t*> childs_;
  vector<Branch_t*> mothers_;
  vector<Branch_t*> disjoints_;
  uint8_t family;
  uint8_t nb_mothers_;

  Branch_t(string value) : family(0), nb_mothers_(0)
    {value_ = value; };
};

class TreeDrawer;
class TreeProperty;

class TreeObject
{
  friend TreeDrawer;
  friend TreeProperty;
public:
  TreeObject() {}
  ~TreeObject();

  void add(string value, vector<string>& mothers, vector<string>& disjoints);
  void add(vector<string>& disjoints);
  void close();

  set<string> getDown(string value);
  set<string> getUp(string value);
  set<string> getDisjoint(string value);

private:
  vector<Branch_t*> branchs_;
  vector<Branch_t*> roots_;

  vector<Branch_t*> tmp_mothers_;

  int depth_;

  void link();
  void add_family(Branch_t* branch, uint8_t family);

  set<string> getDown(Branch_t* branch, string value);
  set<string> getUp(Branch_t* branch, string value);
};

#endif /* TREEOBJECT_H */
