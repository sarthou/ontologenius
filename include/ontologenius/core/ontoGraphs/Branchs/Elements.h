#ifndef ONTOLOGENIUS_ELEMENTS_H
#define ONTOLOGENIUS_ELEMENTS_H

#include <iostream>
#include <ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h>
#include <vector>

namespace ontologenius {

  class ProbabilisticElement
  {
  public:
    float probability;
    bool infered;
    std::vector<TripletsInterface*> induced_traces;

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

    explicit SingleElement(const T& elem, float probability = 1.0, bool infered = false)
    {
      this->elem = elem;
      this->probability = probability;
      this->infered = infered;
    }

    SingleElement(const SingleElement& other, const T& elem)
    {
      this->elem = elem;
      this->probability = other.probability;
      this->infered = other.infered;
    }

    SingleElement(const SingleElement& other)
    {
      // A copy constructor with a pointer is dangerous, never delete it
      // it should be managed by a Graph class
      this->elem = other.elem;
      this->probability = other.probability;
      this->infered = other.infered;
      this->induced_traces = other.induced_traces;
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

    PairElement(const T& first, const U& second, float probability = 1.0, bool infered = false)
    {
      this->first = first;
      this->second = second;
      this->probability = probability;
      this->infered = infered;
    }

    PairElement(const PairElement& other, const T& first, const U& second)
    {
      this->first = first;
      this->second = second;
      this->probability = other.probability;
      this->infered = other.infered;
    }

    PairElement(const PairElement& other)
    {
      // A copy constructor with a pointer is dangerous, never delete it
      // it should be managed by a Graph class
      this->first = other.first;
      this->second = other.second;
      this->probability = other.probability;
      this->infered = other.infered;
      this->induced_traces = other.induced_traces;
    }

    bool operator==(const PairElement& other) const
    {
      return ((first == other.first) && (second == other.second));
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_ELEMENTS_H
