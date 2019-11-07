#ifndef ONTOLOGENIUS_ELEMENTS_H
#define ONTOLOGENIUS_ELEMENTS_H

#include <vector>
#include <iostream>

namespace ontologenius {

class ProbabilisticElement_t
{
public:
  float probability;

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

  Single_t(T elem, float probability = 1.0)
  {
    this->elem = elem;
    this->probability = probability;
  }

  bool operator==(const Single_t& other)
  {
    if(elem == other.elem)
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

  Pair_t(T first, U second, float probability = 1.0)
  {
    this->first = first;
    this->second = second;
    this->probability = probability;
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
