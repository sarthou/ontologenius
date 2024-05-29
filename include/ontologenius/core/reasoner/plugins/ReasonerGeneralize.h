#ifndef ONTOLOGENIUS_REASONERGENERALIZE_H
#define ONTOLOGENIUS_REASONERGENERALIZE_H

#include <tuple>

#include "ontologenius/core/reasoner/plugins/ReasonerInterface.h"

#define DEFAULT_MIN_COUNT 2
#define DEFAULT_MIN_PERCEPT 0.75

namespace ontologenius {

  class ReasonerGeneralize : public ReasonerInterface
  {
  public:
    ReasonerGeneralize();
    ~ReasonerGeneralize() override = default;

    void setParameter(const std::string& name, const std::string& value) override;

    bool periodicReason() override;

    bool implementPeriodicReasoning() override { return true; }

    std::string getName() override;
    std::string getDescription() override;

    bool defaultActive() override { return false; }

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

  template<typename B, typename P>
  class PropertiesCounter
  {
  public:
    PropertiesCounter(int min_count = DEFAULT_MIN_COUNT, float min_percent = DEFAULT_MIN_PERCEPT)
    {
      min_count_ = min_count;
      min_percent_ = min_percent;
    }

    void add(B propertie, P data);
    std::vector<std::tuple<B, P, float>> get();

  private:
    std::vector<B> properties_;
    std::vector<P> datas_;
    std::vector<size_t> counts_;

    size_t min_count_;
    double min_percent_;
  };

  template<typename B, typename P>
  void PropertiesCounter<B, P>::add(B propertie, P data)
  {
    std::vector<size_t> indexs;
    for(size_t i = 0; i < properties_.size(); i++)
      if(properties_[i] == propertie)
        indexs.push_back(i);

    if(indexs.empty())
    {
      properties_.push_back(propertie);
      datas_.push_back(data);
      counts_.push_back(1);
    }
    else
    {
      int index = -1;
      for(auto idx : indexs)
        if(datas_[idx] == data)
        {
          index = idx;
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

  template<typename B, typename P>
  std::vector<std::tuple<B, P, float>> PropertiesCounter<B, P>::get()
  {
    std::vector<std::tuple<B, P, float>> res;
    if(properties_.size() > 0)
    {
      std::vector<B> properties_set;
      std::vector<std::vector<size_t>> index_set;
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
          if(counts_[index_set[i][j]] >= min_count_)
            if(counts_[index_set[i][j]] / (double)counts_set[i] >= min_percent_)
              res.emplace_back(properties_set[i], datas_[index_set[i][j]], counts_[index_set[i][j]] / (double)counts_set[i]);
      }
    }

    return res;
  }

} // namespace ontologenius

#endif // ONTOLOGENIUS_REASONERGENERALIZE_H
