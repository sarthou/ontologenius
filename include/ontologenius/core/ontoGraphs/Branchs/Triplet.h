#ifndef ONTOLOGENIUS_TRIPLET_H
#define ONTOLOGENIUS_TRIPLET_H

#include <algorithm>
#include <string>
#include <vector>

namespace ontologenius {

  struct TripletStr_t
  {
    TripletStr_t(const std::string& subject,
                 const std::string& predicat,
                 const std::string& object,
                 bool add = true) : subject(subject),
                                    predicate(predicat),
                                    object(object),
                                    add(add)
    {}

    bool operator==(const TripletStr_t& other) const
    {
      return ((subject == other.subject) &&
              (predicate == other.predicate) &&
              (object == other.object) &&
              (add == other.add));
    }

    bool valid() const
    {
      return ((subject.empty() == false) &&
              (predicate.empty() == false) &&
              (object.empty() == false));
    }

    std::string toString() const
    {
      return (add ? "[add]" : "[del]") + subject + "|" + predicate + "|" + object;
    }

    std::string subject;
    std::string predicate;
    std::string object;
    bool add;
  };

  template<typename S, typename P, typename O>
  struct Triplet_t
  {
    Triplet_t(S* s, P* p, O* o) : subject(s), predicate(p), object(o) {}
    S* subject;
    P* predicate;
    O* object;

    bool operator==(const Triplet_t& other)
    {
      return ((subject == other.subject) &&
              (predicate == other.predicate) &&
              (object == other.object));
    }

    bool equals(S* s, P* p, O* o)
    {
      return ((subject == s) &&
              (predicate == p) &&
              (object == o));
    }
  };

  class TripletsInterface
  {
  public:
    virtual ~TripletsInterface() = default;
    virtual bool eraseGeneric(void* s, void* p, void* o) = 0;
  };

  template<typename S, typename P, typename O>
  class Triplets : public TripletsInterface
  {
  public:
    ~Triplets() override = default;

    Triplet_t<S, P, O> push(S* subject,
                            P* predicate,
                            O* object)
    {
      triplets.emplace_back(subject, predicate, object);
      return triplets.back();
    }

    bool exist(S* subject,
               P* predicate,
               O* object)
    {
      return (std::find_if(triplets.begin(), triplets.end(),
                           [subject, predicate, object](auto& triplet) { return triplet.equals(subject, predicate, object); }) != triplets.end());
    }

    size_t size()
    {
      return triplets.size();
    }

    size_t find(S* subject,
                P* predicate,
                O* object)
    {
      for(size_t i = 0; i < triplets.size(); i++)
      {
        if(triplets[i].equals(subject, predicate, object))
          return i;
      }
      return -1;
    }

    bool eraseGeneric(void* s, void* p, void* o) override
    {
      return erase((S*)s, (P*)p, (O*)o);
    }

    bool erase(S* subject,
               P* predicate,
               O* object)
    {
      size_t index = this->find(subject, predicate, object);
      if(index != size_t(-1))
      {
        triplets.erase(triplets.begin() + index);
        return true;
      }
      else
        return false;
    }

    std::vector<Triplet_t<S, P, O>> triplets;
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_TRIPLET_H