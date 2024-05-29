#ifndef ONTOLOGENIUS_OBJECTPROPERTIESOWLWRITER_H
#define ONTOLOGENIUS_OBJECTPROPERTIESOWLWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/Owl/writers/PropertiesOwlWriter.h"

namespace ontologenius {

  class ObjectPropertyGraph;
  class ObjectPropertyBranch;

  class ObjectPropertiesOwlWriter : public PropertiesOwlWriter<ObjectPropertyBranch>
  {
  public:
    ObjectPropertiesOwlWriter(ObjectPropertyGraph* property_graph, const std::string& ns);
    ~ObjectPropertiesOwlWriter() = default;

    void write(FILE* file);

  private:
    ObjectPropertyGraph* property_graph_;

    void writeProperty(ObjectPropertyBranch* branch);
    void writeSubPropertyOf(ObjectPropertyBranch* branch);
    void writeInverseOf(ObjectPropertyBranch* branch);
    void writeRange(ObjectPropertyBranch* branch);
    void writeDomain(ObjectPropertyBranch* branch);
    void writeChain(ObjectPropertyBranch* branch);
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_OBJECTPROPERTIESOWLWRITER_H
