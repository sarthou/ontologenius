#ifndef ONTOLOGENIUS_SUBSCRIPTIONPATTERN_H
#define ONTOLOGENIUS_SUBSCRIPTIONPATTERN_H

#include <regex>
#include <string>

#include "ontologenius/core/ontoGraphs/Branchs/Triplet.h"

namespace ontologenius {

  class SubscriptionPattern
  {
  public:
    SubscriptionPattern(const std::string& subject,
                        const std::string& predicat,
                        const std::string& object,
                        bool add) : triplet_(subject, predicat, object, add),
                                    operator_is_undefined_(false)
    {
      init();
    }

    SubscriptionPattern(const std::string& subject,
                        const std::string& predicat,
                        const std::string& object) : triplet_(subject, predicat, object, true),
                                                     operator_is_undefined_(true)

    {
      init();
    }

    explicit SubscriptionPattern(const TripletStr_t& triplet) : triplet_(triplet),
                                                                operator_is_undefined_(false)
    {
      init();
    }

    SubscriptionPattern(const SubscriptionPattern& other) = default;

    static SubscriptionPattern deserialize(const std::string& str)
    {
      if(std::regex_match(str, match, regex))
      {
        if(match[1].str() == "?")
          return SubscriptionPattern(match[2].str(), match[3].str(), match[4].str());
        else
          return SubscriptionPattern(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
      }
      else if(std::regex_match(str, match, regex2))
      {
        if(match[1].str() == "?")
          return SubscriptionPattern(match[2].str(), match[3].str(), match[4].str());
        else
          return SubscriptionPattern(match[2].str(), match[3].str(), match[4].str(), (match[1].str() == "ADD") || (match[1].str() == "add"));
      }
      else
        return SubscriptionPattern(TripletStr_t("", "", "")); // invalid triplet
    }

    bool fit(const TripletStr_t& other) const
    {
      return (((triplet_.add == other.add) || operator_is_undefined_) &&
              ((triplet_.subject == other.subject) || subject_is_undefined_) &&
              ((triplet_.predicate == other.predicate) || predicat_is_undefined_) &&
              ((triplet_.object == other.object) || object_is_undefined_));
    }

    void setSubjectAsClass() { subject_is_indiv_ = false; }
    void setSubjectAsIndividual() { subject_is_indiv_ = true; }
    void setObjectAsClass() { object_is_indiv_ = false; }
    void setObjectAsIndividual() { object_is_indiv_ = true; }
    void setPredicatAsDataProperty() { predicat_is_object_property_ = false; }
    void setPredicatAsObjectProperty() { predicat_is_object_property_ = true; }

    bool isSubjectClass() const { return !subject_is_indiv_; }
    bool isSubjectIndividual() const { return subject_is_indiv_; }
    bool isObjectClass() const { return !object_is_indiv_; }
    bool isObjectIndividual() const { return object_is_indiv_; }
    bool isPredicatDataProperty() const { return !predicat_is_object_property_; }
    bool isPredicatObjectProperty() const { return predicat_is_object_property_; }

    bool isOperatorUndefined() const { return operator_is_undefined_; }
    bool isSubjectUndefined() const { return subject_is_undefined_; }
    bool isObjectUndefined() const { return object_is_undefined_; }
    bool isPredicatUndefined() const { return predicat_is_undefined_; }

    bool add() const { return triplet_.add; }
    const std::string& subject() const { return triplet_.subject; }
    const std::string& predicate() const { return triplet_.predicate; }
    const std::string& object() const { return triplet_.object; }
    const TripletStr_t& getTriplet() const { return triplet_; }

    bool valid() const { return triplet_.valid(); }

  protected:
    TripletStr_t triplet_;

    bool subject_is_indiv_;
    bool object_is_indiv_;
    bool predicat_is_object_property_;

    bool operator_is_undefined_;
    bool subject_is_undefined_;
    bool object_is_undefined_;
    bool predicat_is_undefined_;

    static std::regex regex;
    static std::regex regex2;
    static std::smatch match;

  private:
    void init()
    {
      subject_is_indiv_ = true;
      object_is_indiv_ = true;
      predicat_is_object_property_ = true;

      subject_is_undefined_ = (triplet_.subject == "?");
      object_is_undefined_ = (triplet_.object == "?");
      predicat_is_undefined_ = (triplet_.predicate == "?");
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_SUBSCRIPTIONPATTERN_H
