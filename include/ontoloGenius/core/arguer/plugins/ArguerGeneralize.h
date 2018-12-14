#ifndef ARGUERGENERALIZE_H
#define ARGUERGENERALIZE_H

#include "ontoloGenius/core/arguer/plugins/ArguerInterface.h"

class ArguerGeneralize : public ArguerInterface
{
public:
  ArguerGeneralize();
  ~ArguerGeneralize() {}

  virtual void periodicReason();

  virtual std::string getName();
  virtual std::string getDesciption();

  virtual bool defaultAvtive() {return true;}
private:
  size_t current_id_;
  size_t class_per_period_;

  void setDeduced(ClassBranch_t* me, std::vector<DataPropertyBranch_t*> properties, std::vector<std::string> datas);
  void setDeduced(ClassBranch_t* me, std::vector<ObjectPropertyBranch_t*> properties, std::vector<ClassBranch_t*> datas);
};

/***********
*
* PropertiesCounter
*
*************/

template <typename B, typename P>
class PropertiesCounter
{
public:
  PropertiesCounter()
  {
    min_count = 2;
    min_percent = 0.75;
  }

  void add(B propertie, P data);
  void get(std::vector<B>& properties, std::vector<P>& datas);

private:
  std::vector<B> properties_;
  std::vector<P> datas_;
  std::vector<size_t> counts_;

  size_t min_count;
  double min_percent;
};

template <typename B, typename P>
void PropertiesCounter<B,P>::add(B propertie, P data)
{
  std::vector<size_t> indexs;
  for(size_t i = 0; i < properties_.size(); i++)
    if(properties_[i] == propertie)
      indexs.push_back(i);

  if(indexs.size() == 0)
  {
    properties_.push_back(propertie);
    datas_.push_back(data);
    counts_.push_back(1);
  }
  else
  {
    int index = -1;
    for(size_t i = 0; i < indexs.size(); i++)
      if(datas_[indexs[i]] == data)
      {
        index = indexs[i];
        break;
      }

    if(index == -1)
    {
      properties_.push_back(propertie);
      datas_.push_back(data);
      counts_.push_back(1);
    }
    else
      counts_[index]++;
  }
}

template <typename B, typename P>
void PropertiesCounter<B,P>::get(std::vector<B>& properties, std::vector<P>& datas)
{
  if(properties_.size() > 0)
  {
    std::vector<B> properties_set;
    std::vector<std::vector<size_t> > index_set;
    std::vector<size_t> counts_set;

    for(size_t i = 0; i < properties_.size(); i++)
    {
      int index = -1;
      for(size_t j = 0; j < properties_set.size(); j++)
        if(properties_set[j] == properties_[i])
        {
          index = j;
          break;
        }

      if(index == -1)
      {
        properties_set.push_back(properties_[i]);
        std::vector<size_t> index_tmp;
        index_tmp.push_back(i);
        index_set.push_back(index_tmp);
        counts_set.push_back(counts_[i]);
      }
      else
      {
        index_set[index].push_back(i);
        counts_set[index] += counts_[i];
      }
    }

    for(size_t i = 0; i < properties_set.size(); i++)
    {
      for(size_t j = 0; j < index_set[i].size(); j++)
        if(counts_[index_set[i][j]] >= min_count)
          if(counts_[index_set[i][j]] / (double)counts_set[i] >= min_percent)
          {
            properties.push_back(properties_set[i]);
            datas.push_back(datas_[index_set[i][j]]);
          }
    }
  }
}

#endif
