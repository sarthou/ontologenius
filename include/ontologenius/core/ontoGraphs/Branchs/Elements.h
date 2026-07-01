#ifndef ONTOLOGENIUS_ELEMENTS_H
#define ONTOLOGENIUS_ELEMENTS_H

#include <iostream>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h"
#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"

namespace ontologenius {

  class InferenceRuleNode;

  class ProbabilisticElement
  {
  public:
    float probability;
    bool inferred;
    std::vector<std::string> explanation;
    InferenceRuleNode* used_rule = nullptr;
    std::vector<TripletsInterface*> induced_traces;

    std::string getExplanation()
    {
      std::string res;
      for(const auto& exp : explanation)
      {
        if(res.empty() == false)
          res += ", ";
        res += exp;
      }
      return res;
    }

    bool operator>(float prob) const
    {
      return (probability > prob);
    }

    bool operator<(float prob) const
    {
      return (probability < prob);
    }
  };

  template<typename T>
  class SingleElement : public ProbabilisticElement
  {
  public:
    T elem;

    explicit SingleElement(const T& element,
                           float probability = 1.0,
                           bool inferred = false) : elem(element)
    {
      this->probability = probability;
      this->inferred = inferred;
    }

    SingleElement(const SingleElement& other,
                  const T& element) : elem(element)
    {
      this->probability = other.probability;
      this->inferred = other.inferred;
      this->induced_traces = other.induced_traces;
      this->used_rule = other.used_rule;
      this->explanation = other.explanation;
    }

    SingleElement(const SingleElement& other) : elem(other.elem)
    {
      // A copy constructor with a pointer is dangerous, never delete it
      // it should be managed by a Graph class
      this->probability = other.probability;
      this->inferred = other.inferred;
      this->induced_traces = other.induced_traces;
      this->used_rule = other.used_rule;
      this->explanation = other.explanation;
    }

    SingleElement& operator=(const SingleElement& other)
    {
      this->elem = other.elem;
      this->probability = other.probability;
      this->inferred = other.inferred;
      this->induced_traces = other.induced_traces;
      this->used_rule = other.used_rule;
      this->explanation = other.explanation;
      return *this;
    }

    bool operator==(const SingleElement& other) const
    {
      return (elem == other.elem);
    }

    bool operator==(const T& other) const
    {
      return (elem == other);
    }
  };

  template<typename T, typename U>
  class PairElement : public ProbabilisticElement
  {
  public:
    T first;
    U second;

    PairElement(const T& first_elem,
                const U& second_elem,
                float probability = 1.0,
                bool inferred = false) : first(first_elem),
                                         second(second_elem)
    {
      this->probability = probability;
      this->inferred = inferred;
    }

    PairElement(const PairElement& other,
                const T& first_elem,
                const U& second_elem) : first(first_elem),
                                        second(second_elem)
    {
      this->probability = other.probability;
      this->inferred = other.inferred;
      this->induced_traces = other.induced_traces;
      this->used_rule = other.used_rule;
      this->explanation = other.explanation;
    }

    PairElement(const PairElement& other) : first(other.first),
                                            second(other.second)
    {
      // A copy constructor with a pointer is dangerous, never delete it
      // it should be managed by a Graph class
      this->probability = other.probability;
      this->inferred = other.inferred;
      this->induced_traces = other.induced_traces;
      this->used_rule = other.used_rule;
      this->explanation = other.explanation;
    }

    PairElement& operator=(const PairElement& other)
    {
      // A copy constructor with a pointer is dangerous, never delete it
      // it should be managed by a Graph class
      this->first = other.first;
      this->second = other.second;
      this->probability = other.probability;
      this->inferred = other.inferred;
      this->induced_traces = other.induced_traces;
      this->used_rule = other.used_rule;
      this->explanation = other.explanation;

      return *this;
    }

    bool operator==(const PairElement& other) const
    {
      return ((first == other.first) && (second == other.second));
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ELEMENTS_H
