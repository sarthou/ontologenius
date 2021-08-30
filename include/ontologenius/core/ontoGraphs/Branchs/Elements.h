#ifndef ONTOLOGENIUS_ELEMENTS_H
#define ONTOLOGENIUS_ELEMENTS_H

#include <vector>
#include <iostream>

namespace ontologenius {

class ProbabilisticElement_t
{
public:
  float probability;
  bool infered;

  bool operator>(float prob)
  {
    if(probability > prob)
      return true;
    return false;
  }

  bool operator<(float prob)
  {
    if(probability < prob)
      return true;
    return false;
  }
};

template <typename T>
class Single_t : public ProbabilisticElement_t
{
public:
  T elem;

  Single_t(T elem, float probability = 1.0, bool infered = false)
  {
    this->elem = elem;
    this->probability = probability;
    this->infered = infered;
  }

  Single_t(const Single_t& other, T elem)
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
  }

  bool operator==(const Single_t& other)
  {
    if(elem == other.elem)
      return true;
    return false;
  }

  bool operator==(const T& other)
  {
    if(elem == other)
      return true;
    return false;
  }
};

template <typename T, typename U>
class Pair_t : public ProbabilisticElement_t
{
public:
  T first;
  U second;

  Pair_t(T first, U second, float probability = 1.0, bool infered = false)
  {
    this->first = first;
    this->second = second;
    this->probability = probability;
    this->infered = infered;
  }

  Pair_t(const Pair_t& other, T first, U second)
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
  }

  bool operator==(const Pair_t& other)
  {
    if((first == other.first) && (second == other.second))
      return true;
    return false;
  }
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_ELEMENTS_H
