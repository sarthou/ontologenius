#ifndef ONTOLOGENIUS_REASONERGENERALIZE_H
#define ONTOLOGENIUS_REASONERGENERALIZE_H

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

#include <tuple>

namespace ontologenius {

class ReasonerGeneralize : public ReasonerInterface
{
public:
  ReasonerGeneralize();
  virtual ~ReasonerGeneralize() = default;

  virtual void setParameter(const std::string& name, const std::string& value) override;

  virtual bool periodicReason() override;

  virtual bool implementPeriodicReasoning() override { return true; }

  virtual std::string getName() override;
  virtual std::string getDescription() override;

  virtual bool defaultActive() override {return false;}
private:
  size_t current_id_;
  size_t class_per_period_;

  size_t min_count_;
  float min_percent_;

  void setDeduced(ClassBranch_t* me, std::vector<std::tuple<DataPropertyBranch_t*, LiteralNode*, float>> properties);
  void setDeduced(ClassBranch_t* me, std::vector<std::tuple<ObjectPropertyBranch_t*, ClassBranch_t*, float>> properties);
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
  PropertiesCounter(int min_count = 2, float min_percent = 0.75)
  {
    this->min_count = min_count;
    this->min_percent = min_percent;
  }

  void add(B propertie, P data);
  std::vector<std::tuple<B, P, float>> get();

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
std::vector<std::tuple<B, P, float>> PropertiesCounter<B,P>::get()
{
  std::vector<std::tuple<B, P, float>> res;
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
            res.emplace_back(properties_set[i], datas_[index_set[i][j]], counts_[index_set[i][j]] / (double)counts_set[i]);
    }
  }

  return res;
}

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERGENERALIZE_H
