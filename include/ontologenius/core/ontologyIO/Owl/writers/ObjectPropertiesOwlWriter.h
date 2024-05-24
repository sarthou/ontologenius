#ifndef ONTOLOGENIUS_OBJECTPROPERTIESOWLWRITER_H
#define ONTOLOGENIUS_OBJECTPROPERTIESOWLWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/Owl/writers/PropertiesOwlWriter.h"

namespace ontologenius {

  class ObjectPropertyGraph;
  class ObjectPropertyBranch_t;

  class ObjectPropertiesOwlWriter : public PropertiesOwlWriter<ObjectPropertyBranch_t>
  {
  public:
    ObjectPropertiesOwlWriter(ObjectPropertyGraph* property_graph, const std::string& ns);
    ~ObjectPropertiesOwlWriter(){};

    void write(FILE* file);

  private:
    ObjectPropertyGraph* property_graph_;

    void writeProperty(ObjectPropertyBranch_t* branch);
    void writeSubPropertyOf(ObjectPropertyBranch_t* branch);
    void writeInverseOf(ObjectPropertyBranch_t* branch);
    void writeRange(ObjectPropertyBranch_t* branch);
    void writeDomain(ObjectPropertyBranch_t* branch);
    void writeChain(ObjectPropertyBranch_t* branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJECTPROPERTIESOWLWRITER_H
