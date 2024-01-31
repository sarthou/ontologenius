#include "ontologenius/API/ontologenius/OntologyManipulator.h"
#include "ontologenius/graphical/Display.h"

namespace onto {

OntologyManipulator::OntologyManipulator(const std::string& name) : name_(name),
                                                                    individuals(name),
                                                                    objectProperties(name),
                                                                    dataProperties(name),
                                                                    classes(name),
                                                                    actions(name),
                                                                    reasoners(name),
                                                                    feeder(name),
                                                                    sparql(name)
{
  sparql.client_.wait(-1);
}

OntologyManipulator::OntologyManipulator(const OntologyManipulator& other): name_(other.name_),
                                                                      individuals(other.name_),
                                                                      objectProperties(other.name_),
                                                                      dataProperties(other.name_),
                                                                      classes(other.name_),
                                                                      actions(other.name_),
                                                                      reasoners(other.name_),
                                                                      feeder(other.name_),
                                                                      sparql(other.name_)
{
  sparql.client_.wait(-1);
}

OntologyManipulator::OntologyManipulator(OntologyManipulator&& other): name_(other.name_),
                                                                      individuals(other.name_),
                                                                      objectProperties(other.name_),
                                                                      dataProperties(other.name_),
                                                                      classes(other.name_),
                                                                      actions(other.name_),
                                                                      reasoners(other.name_),
                                                                      feeder(other.name_),
                                                                      sparql(other.name_)
{
  sparql.client_.wait(-1);
}

} // namespace onto