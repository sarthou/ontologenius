#ifndef ONTOLOGENIUS_ELEMENTS_H
#define ONTOLOGENIUS_ELEMENTS_H

#include <vector>

namespace ontologenius {

class ProbabilisticElement_t
{
public:
  float propability;

  bool operator>(float prob)
  {
    if(propability > prob)
      return true;
    return false;
  }

  bool operator<(float prob)
  {
    if(propability < prob)
      return true;
    return false;
  }
};

template <typename T>
class Single_t : public ProbabilisticElement_t
{
public:
  T elem;

  Single_t(T elem, float propability = 1.0)
  {
    this->elem = elem;
    this->propability = propability;
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

  Pair_t(T first, U second, float propability = 1.0)
  {
    this->first = first;
    this->second = second;
    this->propability = propability;
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
