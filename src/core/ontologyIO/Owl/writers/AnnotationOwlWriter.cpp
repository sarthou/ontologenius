#include "ontologenius/core/ontologyIO/Owl/writers/AnnotationOwlWriter.h"

#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/DataPropertyBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/LiteralNode.h"
#include "ontologenius/core/ontoGraphs/Branchs/ObjectPropertyBranch.h"

namespace ontologenius {

  AnnotationOwlWriter::AnnotationOwlWriter(ObjectPropertyGraph* object_property_graph, DataPropertyGraph* data_property_graph, const std::string& ns)
  {
    object_property_graph_ = object_property_graph;
    data_property_graph_ = data_property_graph;
    ns_ = ns;
  }

  void AnnotationOwlWriter::write(FILE* file)
  {
    file_ = file;

    {
      const std::shared_lock<std::shared_timed_mutex> lock(object_property_graph_->mutex_);

      const std::vector<ObjectPropertyBranch*> properties = object_property_graph_->get();
      for(auto& property : properties)
        if(property->annotation_usage_)
          writeAnnotation(property);
    }

    {
      const std::shared_lock<std::shared_timed_mutex> lock(data_property_graph_->mutex_);

      const std::vector<DataPropertyBranch*> properties = data_property_graph_->get();
      for(auto& property : properties)
        if(property->annotation_usage_)
          writeAnnotation(property);
    }

    file_ = nullptr;
  }

  void AnnotationOwlWriter::writeAnnotation(ObjectPropertyBranch* branch)
  {
    std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:AnnotationProperty rdf:about=\"" +
                      ns_ + "#" + branch->value() + "\">\n";
    writeString(tmp);

    writeSubPropertyOf(branch);
    writeRange(branch->ranges_);
    writeDomain(branch->domains_);

    tmp = "    </owl:AnnotationProperty>\n\n\n\n";
    writeString(tmp);
  }

  void AnnotationOwlWriter::writeAnnotation(DataPropertyBranch* branch)
  {
    std::string tmp = "    <!-- " + ns_ + "#" + branch->value() + " -->\n\n\
    <owl:AnnotationProperty rdf:about=\"" +
                      ns_ + "#" + branch->value() + "\">\n";
    writeString(tmp);

    writeSubPropertyOf(branch);
    writeRange(branch->ranges_);
    writeDomain(branch->domains_);

    tmp = "    </owl:AnnotationProperty>\n\n\n\n";
    writeString(tmp);
  }

  void AnnotationOwlWriter::writeSubPropertyOf(ObjectPropertyBranch* branch)
  {
    for(auto& mother : branch->mothers_)
      if(mother.infered == false)
      {
        const std::string tmp = "        <rdfs:subPropertyOf" +
                                getProba(mother) +
                                " rdf:resource=\"" + ns_ + "#" +
                                mother.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void AnnotationOwlWriter::writeSubPropertyOf(DataPropertyBranch* branch)
  {
    for(auto& mother : branch->mothers_)
      if(mother.infered == false)
      {
        const std::string tmp = "        <rdfs:subPropertyOf" +
                                getProba(mother) +
                                " rdf:resource=\"" + ns_ + "#" +
                                mother.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void AnnotationOwlWriter::writeRange(const std::vector<LiteralNode*>& ranges)
  {
    for(const auto& range : ranges)
    {
      const std::string tmp = "        <rdfs:range rdf:resource=\"" +
                              range->getNs() +
                              "#" +
                              range->type_ +
                              +"\"/>\n";
      writeString(tmp);
    }
  }

  void AnnotationOwlWriter::writeRange(const std::vector<ClassElement>& ranges)
  {
    for(auto range : ranges)
      if(range.infered == false)
      {
        const std::string tmp = "        <rdfs:range" +
                                getProba(range) +
                                " rdf:resource=\"" + ns_ + "#" +
                                range.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

  void AnnotationOwlWriter::writeDomain(const std::vector<ClassElement>& domains)
  {
    for(auto domain : domains)
      if(domain.infered == false)
      {
        const std::string tmp = "        <rdfs:domain" +
                                getProba(domain) +
                                " rdf:resource=\"" + ns_ + "#" +
                                domain.elem->value() + "\"/>\n";
        writeString(tmp);
      }
  }

} // namespace ontologenius
