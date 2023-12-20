#include "ontologenius/API/ontologenius/OntologyManipulatorIndex.h"
#include "ontologenius/graphical/Display.h"

namespace onto {

OntologyManipulatorIndex::OntologyManipulatorIndex(const std::string& name) : name_(name),
                                                                    individuals(name),
                                                                    objectProperties(name),
                                                                    dataProperties(name),
                                                                    classes(name),
                                                                    actions(name),
                                                                    reasoners(name),
                                                                    feeder(name),
                                                                    sparql(name),
                                                                    conversion(name)
{
  conversion.client_.wait(-1);
}

OntologyManipulatorIndex::OntologyManipulatorIndex(const OntologyManipulatorIndex& other): name_(other.name_),
                                                                      individuals(other.name_),
                                                                      objectProperties(other.name_),
                                                                      dataProperties(other.name_),
                                                                      classes(other.name_),
                                                                      actions(other.name_),
                                                                      reasoners(other.name_),
                                                                      feeder(other.name_),
                                                                      sparql(other.name_),
                                                                      conversion(other.name_)
{
  conversion.client_.wait(-1);
}

OntologyManipulatorIndex::OntologyManipulatorIndex(OntologyManipulatorIndex&& other): name_(other.name_),
                                                                      individuals(other.name_),
                                                                      objectProperties(other.name_),
                                                                      dataProperties(other.name_),
                                                                      classes(other.name_),
                                                                      actions(other.name_),
                                                                      reasoners(other.name_),
                                                                      feeder(other.name_),
                                                                      sparql(other.name_),
                                                                      conversion(other.name_)
{
  conversion.client_.wait(-1);
}

} // namespace onto
