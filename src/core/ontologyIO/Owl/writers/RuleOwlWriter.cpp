#include "ontologenius/core/ontologyIO/Owl/writers/RuleOwlWriter.h"

#include <cstddef>
#include <cstdio>
#include <shared_mutex>
#include <string>
#include <vector>

#include "ontologenius/core/ontoGraphs/Branchs/AnonymousClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/ClassBranch.h"
#include "ontologenius/core/ontoGraphs/Branchs/RuleBranch.h"
#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/AnonymousClassOwlWriter.h"

namespace ontologenius {

  RuleOwlWriter::RuleOwlWriter(RuleGraph* rule_graph,
                               FILE* file,
                               const std::string& ns) : AnonymousClassOwlWriter("", file, ns),
                                                        rule_graph_(rule_graph)
  {}

  void RuleOwlWriter::write()
  {
    const std::shared_lock<std::shared_timed_mutex> lock(rule_graph_->mutex_);

    const std::vector<RuleBranch*> rules = rule_graph_->get();

    // write all the variables involved in the rules
    for(const auto& var : rule_graph_->getVariables())
      writeVariable(var);

    // write the rules
    for(auto* rule : rules)
      writeRule(rule);
  }

  void RuleOwlWriter::writeRule(RuleBranch* branch)
  {
    const size_t level = 1;
    const std::string field = "rdf:Description";

    writeString("<" + field + ">\n", level);
    writeCommentDictionary(branch);

    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#Imp\"/>\n", level + 1);

    // write the body of the rule
    writeString("<swrl:body>\n", level + 1);
    writeAtom(branch->rule_body_, branch->rule_body_.front(), level + 2);
    writeString("</swrl:body>\n", level + 1);

    // write the head of the rule
    writeString("<swrl:head>\n", level + 1);
    writeAtom(branch->rule_head_, branch->rule_head_.front(), level + 2);
    writeString("</swrl:head>\n", level + 1);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeAtom(const std::vector<RuleTriplet_t>& atom_list, const RuleTriplet_t& current_atom, size_t level, size_t index)
  {
    const std::string field = "rdf:Description";

    writeString("<" + field + ">\n", level);
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#AtomList\"/>\n", level + 1);

    writeString("<rdf:first>\n", level + 1);
    writeString("<" + field + ">\n", level + 2);

    // write the atom
    switch(current_atom.atom_type_)
    {
    case rule_atom_class:
      writeClassAtom(current_atom, level + 3);
      break;
    case rule_atom_object:
      writeObjectAtom(current_atom, level + 3);
      break;
    case rule_atom_data:
      writeDataAtom(current_atom, level + 3);
      break;
    case rule_atom_builtin:
      writeBuiltinAtom(current_atom, level + 3);
      break;
    default:
      break;
    }

    writeString("</" + field + ">\n", level + 2);
    writeString("</rdf:first>\n", level + 1);

    index++;
    if(index < atom_list.size())
    {
      writeString("<rdf:rest>\n", level + 1);
      writeAtom(atom_list, atom_list[index], level + 2, index);
      writeString("</rdf:rest>\n", level + 1);
    }
    else
      writeString("<rdf:rest rdf:resource=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#nil\"/>\n", level + 1);

    writeString("</" + field + ">\n", level);
  }

  void RuleOwlWriter::writeClassAtom(const RuleTriplet_t& class_atom, size_t level)
  {
    const std::string field_name = "swrl:classPredicate";

    // writing the Atom Type
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#ClassAtom\"/>\n", level);
    if(class_atom.anonymous_element == nullptr)
      writeString("<" + field_name + " " + getRdfResource(class_atom.class_predicate->value()) + "/>\n", level + 1);
    else if(class_atom.anonymous_element->ano_trees_.empty() == false)
    {
      writeString("<" + field_name + ">\n", level);
      auto* root_node = class_atom.anonymous_element->ano_trees_.front()->root_node_;
      writeClassExpression(root_node, level + 1);
      writeString("</" + field_name + ">\n", level);
    }

    writeRuleArguments(class_atom.arguments, level);
  }

  void RuleOwlWriter::writeObjectAtom(const RuleTriplet_t& object_atom, size_t level)
  {
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#IndividualPropertyAtom\"/>\n", level);
    writeString("<swrl:propertyPredicate " + getRdfResource(object_atom.object_predicate->value()) + "/>\n", level);
    writeRuleArguments(object_atom.arguments, level);
  }

  void RuleOwlWriter::writeDataAtom(const RuleTriplet_t& data_atom, size_t level)
  {
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#DatavaluedPropertyAtom\"/>\n", level);
    writeString("<swrl:propertyPredicate " + getRdfResource(data_atom.data_predicate->value()) + "/>\n", level);
    writeRuleArguments(data_atom.arguments, level);
  }

  void RuleOwlWriter::writeBuiltinAtom(const RuleTriplet_t& builtin_atom, size_t level)
  {
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#BuiltinAtom\"/>\n", level);
    writeString("<swrl:builtin rdf:resource=\"http://www.w3.org/2003/11/swrlb#" + builtin_atom.builtinToString() + "\"/>\n", level);
    writeRuleBuiltinArguments(builtin_atom.arguments, 0, level);
  }

  void RuleOwlWriter::writeRuleBuiltinArguments(const std::vector<RuleArgument_t>& arguments, size_t index, size_t level)
  {
    if(index >= arguments.size())
    {
      writeString("<rdf:rest rdf:resource=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#nil\"/>", level);
      return;
    }

    bool use_collection = arguments.at(index).is_variable;
    if(use_collection)
    {
      for(size_t i = index + 1; i < arguments.size(); i++)
        if(arguments.at(i).is_variable == false)
        {
          use_collection = false;
          break;
        }
    }

    std::string key = (index == 0) ? "swrl:arguments" : "rdf:rest";

    if(use_collection)
    {
      writeString("<" + key + " rdf:parseType=\"Collection\">\n", level);
      for(size_t i = index; i < arguments.size(); i++)
        writeString(getArgumentString(arguments.at(i), "rdf:Description"), level + 1);
      writeString("</" + key + ">\n", level);
    }
    else
    {
      writeString("<" + key + ">\n", level);
      writeString("<rdf:Description>\n", level + 1);
      writeString("<rdf:type rdf:resource=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#List\"/>\n", level + 2);
      writeString(getArgumentString(arguments.at(index), "rdf:first"), level + 2);
      writeRuleBuiltinArguments(arguments, index + 1, level + 2);
      writeString("</rdf:Description>\n", level + 1);
      writeString("</" + key + ">\n", level);
    }
  }

  void RuleOwlWriter::writeRuleArguments(const std::vector<RuleArgument_t>& arguments, size_t level)
  {
    for(size_t i = 0; i < arguments.size(); i++)
      writeString(getArgumentString(arguments.at(i), "swrl:argument" + std::to_string(i + 1)), level);
  }

  std::string RuleOwlWriter::getArgumentString(const RuleArgument_t& arg, const std::string& key)
  {
    if(arg.datatype_value != nullptr)
      return "<" + key + " rdf:datatype=\"" + arg.datatype_value->type_->getNamespace() + "#" + arg.datatype_value->type_->value() + "\">" +
             arg.datatype_value->data() + "</" + key + ">\n";
    else if(arg.indiv_value != nullptr)
      return "<" + key + " " + getRdfResource(arg.indiv_value->value()) + "/>\n";
    else
      return "<" + key + " rdf:resource=\"urn:swrl:var#" + arg.name + "\"/>\n";
  }

  void RuleOwlWriter::writeVariable(const std::string& rule_variable)
  {
    const size_t level = 1;
    std::string field = "rdf:Description";

    writeString("<" + field + " rdf:about=\"urn:swrl:var#" + rule_variable + "\">\n", level);
    writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#Variable\"/>\n", level + 1);
    writeString("</" + field + ">\n", level);
  }

} // namespace ontologenius