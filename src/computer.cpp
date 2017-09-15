#include "ontoloGenius/computer.h"

#include <iostream>
#include <sstream>

using namespace std;

bool computer::compute(string equation, tree& onto)
{
  size_t pos = 0;
  vector<string> LR;
  split(equation, LR, '=');
  if(LR.size() == 2)
  {
    vector<string> tmp;
    split(LR[0], tmp, '|');
    L.resize(tmp.size());
    for(unsigned int i = 0; i < tmp.size(); i++)
      split(tmp[i], L[i], '_');
    tmp.clear();
    split(LR[1], tmp, '|');
    R.resize(tmp.size());
    for(unsigned int i = 0; i < tmp.size(); i++)
      split(tmp[i], R[i], '_');

    for(unsigned int i = 0; i < L.size(); i++)
    {
      finder_t finder;
      finder.find.resize(R.size());
      for(unsigned int i = 0; i < R.size(); i++)
        finder.find[i].resize(R[i].size(), false);
      finder.words.clear();
      finder.words.resize(L[i].size());

      for(unsigned int j = 0; j < L[i].size(); j++)
      {
        cout << "looking for " << L[i][j] << " as : ";
        finder.words[j] = onto.getUp(L[i][j]);
        for(set<string>::iterator it = finder.words[j].begin(); it != finder.words[j].end(); ++it)
          cout << *it + " ";
        cout << endl;

        for(unsigned int orR = 0; orR < R.size(); orR++)
        {
          for(unsigned int andR = 0; andR < R[orR].size(); andR++)
          {
            cout << "-> compare with " << R[orR][andR] << " : ";
            if(finder.words[j].find(R[orR][andR]) == finder.words[j].end())
              cout << " NO" << endl;
            else
            {
              cout << *finder.words[j].find(R[orR][andR]) << endl;
              finder.find[orR][andR] = true;
            }
            cout << "and" << endl;
          }
          cout << "or" << endl;
        }
      }

      bool find = false;
      for(unsigned int i = 0; i < R.size(); i++)
      {
        find = true;
        for(unsigned int j = 0; j < R[i].size(); j++)
          find &= finder.find[i][j];

        if(find)
        {
          cout << "TRUE !!!!!" << endl;
          return true;
        }
      }

      cout << "NEXT" << endl;
    }
  }
  else
    cout << " = error" << endl;

  cout << "FALSE !!!!!" << endl;
  return false;
}

bool computer::split(const string &txt, vector<string> &strs, char ch)
{
  istringstream iss(txt.c_str());
  string s;
  while(getline(iss, s, ch))
    strs.push_back(s);
  if(strs.size() > 1)
    return true;
  else
    return false;
}
