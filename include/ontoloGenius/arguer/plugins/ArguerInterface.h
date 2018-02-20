#ifndef ARGUERINTERFACE_H
#define ARGUERINTERFACE_H

#include "ontoloGenius/ontoGraphs/Ontology.h"
#include <string>

class ArguerInterface
{
public:
  virtual ~ArguerInterface() {}

  virtual void initialize(Ontology* onto) {ontology_ = onto; }

  virtual void preReason() {}
  virtual void postReason() {}

  virtual std::string getName() = 0;
  virtual std::string getDesciption() = 0;
protected:
  ArguerInterface() { }
  
  Ontology* ontology_;
};

#endif
