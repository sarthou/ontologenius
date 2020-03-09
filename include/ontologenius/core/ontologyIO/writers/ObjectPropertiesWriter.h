#ifndef ONTOLOGENIUS_OBJECTPROPERTIESWRITER_H
#define ONTOLOGENIUS_OBJECTPROPERTIESWRITER_H

#include <string>

#include "ontologenius/core/ontologyIO/writers/PropertiesWriter.h"

namespace ontologenius {

class ObjectPropertyGraph;
class ObjectPropertyBranch_t;

class ObjectPropertiesWriter : public PropertiesWriter<ObjectPropertyBranch_t>
{
public:
  ObjectPropertiesWriter(ObjectPropertyGraph* property_graph) {property_graph_ = property_graph; };
  ~ObjectPropertiesWriter() {};

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

#endif // ONTOLOGENIUS_OBJECTPROPERTIESWRITER_H
