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
    std::string service_name = (name_ == "") ? "ontologenius/conversion" : "ontologenius/conversion/" + name_;

    // do we really need to wait for this? By the way `compat::ros::wait_for_service` doesn't work on ROS2 yet
    ontologenius::compat::ros::wait_for_service(service_name);
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
    std::string service_name = (name_ == "") ? "ontologenius/conversion" : "ontologenius/conversion/" + name_;
    ontologenius::compat::ros::wait_for_service(service_name); // same here
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
    std::string service_name = (name_ == "") ? "ontologenius/conversion" : "ontologenius/conversion/" + name_;
    ontologenius::compat::ros::wait_for_service(service_name); // same here
}

} // namespace onto
