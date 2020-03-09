#include "ontologenius/core/Computer.h"

#include <iostream>
#include <sstream>
#include <chrono>

namespace ontologenius {

using namespace std;

bool Computer::compute(std::string equation, ClassGraph& onto)
{
  using namespace std::chrono;

  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  bool invert = false;
  std::vector<std::string> LR;
  split(equation, LR, "!=");
  if(LR.size() > 1)
    invert = true;
  else
  {
    LR.clear();
    split(equation, LR, "==");
  }
  if(LR.size() == 2)
  {
    std::vector<std::string> tmp;
    split(LR[0], tmp, '|');
    L.resize(tmp.size());
    notL.resize(tmp.size());
    for(size_t i = 0; i < tmp.size(); i++)
    {
      split(tmp[i], L[i], '_');
      for(size_t j = 0; j < L[i].size(); j++)
      {
        std::vector<std::string> word;
        split(L[i][j], word, '-');
        L[i][j] = word[0];
        notL[i].resize(L[i].size(), false);
        if(L[i][j][0] == '!')
        {
          L[i][j].erase(0,1);
          notL[i][j] = true;
        }
      }
    }
    tmp.clear();
    split(LR[1], tmp, '|');
    R.resize(tmp.size());
    notR.resize(tmp.size());
    for(size_t i = 0; i < tmp.size(); i++)
    {
      split(tmp[i], R[i], '_');
      for(size_t j = 0; j < R[i].size(); j++)
      {
        std::vector<std::string> word;
        split(R[i][j], word, '-');
        R[i][j] = word[0];
        notR[i].resize(R[i].size(), false);
        if(R[i][j][0] == '!')
        {
          R[i][j].erase(0,1);
          notR[i][j] = true;
        }
      }
    }

    for(size_t i = 0; i < L.size(); i++)
    {
      finder_t finder;
      finder.find.resize(R.size());
      for(size_t i = 0; i < R.size(); i++)
        finder.find[i].resize(R[i].size(), false);
      finder.words.clear();
      finder.words.resize(L[i].size());

      for(size_t j = 0; j < L[i].size(); j++)
      {
        //cout << "looking for " << L[i][j] << " as : ";
        finder.words[j] = onto.getUp(L[i][j]);
        /*for(std::unordered_set<std::string>::iterator it = finder.words[j].begin(); it != finder.words[j].end(); ++it)
          cout << *it + " ";
        if(notL[i][j])
          cout << " as NOT ";
        cout << endl;*/

        for(size_t orR = 0; orR < R.size(); orR++)
        {
          for(size_t andR = 0; andR < R[orR].size(); andR++)
          {
            if(notR[orR][andR])
            {
              //cout << "-> compare with !" << R[orR][andR] << " : ";
              std::unordered_set<std::string> disjoint = onto.getDisjoint(R[orR][andR]);
              for(const std::string& it : disjoint)
              {
                if(finder.words[j].find(it) != finder.words[j].end())
                {
                  //cout << *finder.words[j].find(*it) << endl;
                  finder.find[orR][andR] = true;
                }
              }

              /*if(finder.find[orR][andR] != true)
                cout << " NO" << endl;
              else
                cout << endl;*/
            }
            else
            {
              //cout << "-> compare with " << R[orR][andR] << " : ";
              if(finder.words[j].find(R[orR][andR]) != finder.words[j].end())
              {
                //cout << *finder.words[j].find(R[orR][andR]) << endl;
                finder.find[orR][andR] = true;
              }
              /*else
                cout << " NO" << endl;*/
            }
            //cout << "and" << endl;
          }
          //cout << "or" << endl;
        }
      }

      bool find = false;
      for(size_t i = 0; i < R.size(); i++)
      {
        find = true;
        for(size_t j = 0; j < R[i].size(); j++)
          find &= finder.find[i][j];

        if(invert)
          find = !find;
        if(find)
        {
          high_resolution_clock::time_point t2 = high_resolution_clock::now();

          duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

          std::cout << "It took me " << time_span.count() << " seconds.";
          std::cout << std::endl;

          cout << "TRUE !!!!!" << endl;
          return true;
        }
      }

      cout << "NEXT" << endl;
    }
  }
  else
    cout << " = error" << endl;

  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "It took me " << time_span.count() << " seconds.";
  std::cout << std::endl;

  cout << "FALSE !!!!!" << endl;
  return false;
}

bool Computer::split(const std::string &txt, std::vector<std::string> &strs, char ch)
{
  istringstream iss(txt.c_str());
  std::string s;
  while(getline(iss, s, ch))
    strs.push_back(s);
  if(strs.size() > 1)
    return true;
  else
    return false;
}

bool Computer::split(const std::string &txt, std::vector<std::string> &strs, std::string delim)
{
  std::string text = txt;
  while(text.find(delim) != std::string::npos)
  {
    cout << text << endl;
    size_t pos = text.find(delim);
    std::string part = text.substr(0, pos);
    text = text.substr(pos + delim.size(), text.size() - pos - delim.size());
    strs.push_back(part);
  }
  strs.push_back(text);
  if(strs.size() > 1)
    return true;
  else
    return false;
}

} // namespace ontologenius
