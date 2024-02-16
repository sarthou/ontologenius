#ifndef ONTOLOGENIUS_ELEMENTS_H
#define ONTOLOGENIUS_ELEMENTS_H

#include <vector>
#include <iostream>
#include <ontologenius/core/ontoGraphs/Branchs/RelationsWithInductions.h>

namespace ontologenius {

class ProbabilisticElement_t
{
public:
  float probability;
  bool infered;
  std::vector<TripletsInterface*> induced_traces;

  bool operator>(float prob)
  {
    return (probability > prob);
  }

  bool operator<(float prob)
  {
    return (probability < prob);
  }
};

template <typename T>
class Single_t : public ProbabilisticElement_t
{
public:
  T elem;

  explicit Single_t(const T& elem, float probability = 1.0, bool infered = false)
  {
    this->elem = elem;
    this->probability = probability;
    this->infered = infered;
  }

  Single_t(const Single_t& other, const T& elem)
  {
    this->elem = elem;
    this->probability = other.probability;
    this->infered = other.infered;
  }

  Single_t(const Single_t& other)
  {
    // A copy constructor with a pointer is dangerous, never delete it
    // it should be managed by a Graph class
    this->elem = other.elem;
    this->probability = other.probability;
    this->infered = other.infered;
    this->induced_traces = other.induced_traces;
  }

  bool operator==(const Single_t& other)
  {
    return (elem == other.elem);
  }

  bool operator==(const T& other)
  {
    return (elem == other);
  }
};

template <typename T, typename U>
class Pair_t : public ProbabilisticElement_t
{
public:
  T first;
  U second;

  Pair_t(const T& first, const U& second, float probability = 1.0, bool infered = false)
  {
    this->first = first;
    this->second = second;
    this->probability = probability;
    this->infered = infered;
  }

  Pair_t(const Pair_t& other, const T& first, const U& second)
  {
    this->first = first;
    this->second = second;
    this->probability = other.probability;
    this->infered = other.infered;
  }

  Pair_t(const Pair_t& other)
  {
    // A copy constructor with a pointer is dangerous, never delete it
    // it should be managed by a Graph class
    this->first = other.first;
    this->second = other.second;
    this->probability = other.probability;
    this->infered = other.infered;
    this->induced_traces = other.induced_traces;
  }

  bool operator==(const Pair_t& other)
  {
    return ((first == other.first) && (second == other.second));
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ELEMENTS_H
