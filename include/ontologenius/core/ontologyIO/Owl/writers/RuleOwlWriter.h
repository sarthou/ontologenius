#ifndef ONTOLOGENIUS_RULEOWLWRITER_H
#define ONTOLOGENIUS_RULEOWLWRITER_H

#include "ontologenius/core/ontoGraphs/Graphs/RuleGraph.h"
#include "ontologenius/core/ontologyIO/Owl/writers/NodeOwlWriter.h"

namespace ontologenius {

  class RuleGraph;
  class RuleBranch;
  class AnonymousClassElement;

  class RuleOwlWriter : private NodeOwlWriter
  {
  public:
    RuleOwlWriter(RuleGraph* rule_graph, const std::string& ns);
    ~RuleOwlWriter() = default;

    void write(FILE* file);
    void writeGeneralAxioms(FILE* file);

  private:
    RuleGraph* rule_graph_;

    void writeRule(RuleBranch* branch);
    void writeVariable(const std::string& rule_variable);
    void writeRuleImpl(RuleBranch* branch, size_t level);

    void writeAtom(ClassAtom_t* class_atom, size_t level);
    void writeAtom(ObjectPropertyAtom_t* object_atom, size_t level);
    void writeAtom(DataPropertyAtom_t* data_atom, size_t level);
    void writeAtom(BuiltinAtom_t* builtin_atom, size_t level);

    void writeAtomList(const RuleAtomList_t& atom_list, size_t level) // TODO take list as const ref
    {
      size_t type = 0;

      switch(type)
      {
      case 0:
        if(atom_list.class_atoms_.empty() == false)
        {
          writeAtom(atom_list, atom_list.class_atoms_.front(), level, type);
          break;
        }
        else
        {
          type++;
          [[fallthrough]];
        }
      case 1:
        if(atom_list.object_atoms_.empty() == false)
        {
          writeAtom(atom_list, atom_list.object_atoms_.front(), level, type);
          break;
        }
        else
        {
          type++;
          [[fallthrough]];
        }
      case 2:
        if(atom_list.data_atoms_.empty() == false)
        {
          writeAtom(atom_list, atom_list.data_atoms_.front(), level, type);
          break;
        }
        else
        {
          type++;
          [[fallthrough]];
        }
      case 3:
        if(atom_list.builtin_atoms_.empty() == false)
        {
          writeAtom(atom_list, atom_list.builtin_atoms_.front(), level, type);
          break;
        }
        else
        {
          type++;
          [[fallthrough]];
        }
      default:
        break;
      }
    }

    template<typename T>
    void writeAtom(const RuleAtomList_t& atom_list, T* current_atom, size_t level, size_t type = 0, size_t index = 0)
    {
      std::string field = "rdf:Description";

      writeString("<" + field + ">\n", level);

      writeString("<rdf:type rdf:resource=\"http://www.w3.org/2003/11/swrl#AtomList\"/>\n", level + 1);

      writeString("<rdf:first>\n", level + 1);
      writeString("<" + field + ">\n", level + 2);

      // write the atom
      writeAtom(current_atom, level + 3);

      writeString("</" + field + ">\n", level + 2);
      writeString("</rdf:first>\n", level + 1);

      index++;

      switch(type)
      {
      case 0:
        if(index < atom_list.class_atoms_.size())
        {
          writeString("<rdf:rest>\n", level + 1);
          writeAtom(atom_list, atom_list.class_atoms_[index], level + 2, type, index);
          writeString("</rdf:rest>\n", level + 1);
          break;
        }
        else
        {
          index = 0;
          type++;
          [[fallthrough]];
        }
      case 1:
        if(index < atom_list.object_atoms_.size())
        {
          writeString("<rdf:rest>\n", level + 1);
          writeAtom(atom_list, atom_list.object_atoms_[index], level + 2, type, index);
          writeString("</rdf:rest>\n", level + 1);
          break;
        }
        else
        {
          index = 0;
          type++;
          [[fallthrough]];
        }
      case 2:
        if(index < atom_list.data_atoms_.size())
        {
          writeString("<rdf:rest>\n", level + 1);
          writeAtom(atom_list, atom_list.data_atoms_[index], level + 2, type, index);
          writeString("</rdf:rest>\n", level + 1);
          break;
        }
        else
        {
          index = 0;
          type++;
          [[fallthrough]];
        }
      case 3:
        if(index < atom_list.builtin_atoms_.size())
        {
          writeString("<rdf:rest>\n", level + 1);
          writeAtom(atom_list, atom_list.builtin_atoms_[index], level + 2, type, index);
          writeString("</rdf:rest>\n", level + 1);
          break;
        }
        else
        {
          index = 0;
          type++;
          [[fallthrough]];
        }
      default:
        writeString("<rdf:rest rdf:resource=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#nil\"/>\n", level + 1);
        break;
      }

      writeString("</" + field + ">\n", level);
    }

    // Anonymous classes method duplicates
    void
    writeEquivalentClass(ClassBranch* branch);
    void writeRestriction(AnonymousClassElement* ano_elem, size_t level);
    void writeClassExpression(AnonymousClassElement* ano_elem, size_t level);
    void writeDatatypeExpression(AnonymousClassElement* ano_elem, size_t level);
    void writeIntersection(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop = false);
    void writeUnion(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop = false);
    void writeOneOf(AnonymousClassElement* ano_elem, size_t level);
    void writeComplement(AnonymousClassElement* ano_elem, size_t level);
    void writeDataComplement(AnonymousClassElement* ano_elem, size_t level);
    void writeComplexDescription(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop = false);
    void writeCardinalityValue(AnonymousClassElement* ano_elem, size_t level);
    void writeCardinalityRange(AnonymousClassElement* ano_elem, size_t level, bool is_data_prop);
    void writeCardinality(AnonymousClassElement* ano_element, size_t level);

    std::string getResource(AnonymousClassElement* ano_elem, const std::string& attribute_name = "rdf:resource", bool used_property = false);
  };
} // namespace ontologenius

#endif // ONTOLOGENIUS_RULEOWLWRITER_H