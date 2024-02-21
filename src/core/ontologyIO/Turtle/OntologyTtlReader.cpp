#include "ontologenius/core/ontologyIO/Turtle/OntologyTtlReader.h"

#include <fstream>

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/utility/error_code.h"
#include "ontologenius/graphical/Display.h"

namespace ontologenius {

OntologyTtlReader::OntologyTtlReader(ClassGraph* class_graph,
                                     ObjectPropertyGraph* object_property_graph,
                                     DataPropertyGraph* data_property_graph,
                                     IndividualGraph* individual_graph,
                                     AnonymousClassGraph* anonymous_graph) : OntologyReader(class_graph,
                                                                                         object_property_graph,
                                                                                         data_property_graph,
                                                                                         individual_graph,
                                                                                         anonymous_graph),
                                                                          double_reg_("^[-+]?\\d+\\.\\d+[eE][-+]?\\d+$"),
                                                                          decimal_reg_("^[-+]?\\d*\\.\\d+$"),
                                                                          integer_reg_("^[-+]?\\d+$")
{}

OntologyTtlReader::OntologyTtlReader(Ontology& onto) : OntologyReader(onto),
                                                       double_reg_("^[-+]?\\d+\\.\\d+[eE][-+]?\\d+$"),
                                                       decimal_reg_("^[-+]?\\d*\\.\\d+$"),
                                                       integer_reg_("^[-+]?\\d+$")
{}

int OntologyTtlReader::readFromUri(std::string& content, const std::string& uri)
{
  return read(content, uri);
}

int OntologyTtlReader::readFromFile(const std::string& file_name)
{
  std::ifstream f(file_name);

  if(!f.is_open())
  {
    Display::error("Fail to open : " + file_name);
    return -1;
  }

  f.seekg(0, std::ios::end);
  const auto size = f.tellg();

  std::string response;
  response.resize(size);
  f.seekg(0);
  f.read(&response[0], size); 
  f.close();
  
  return read(response, file_name);
}

int OntologyTtlReader::read(std::string& raw_turtle, const std::string& file_name)
{
  raw_turtle = " " + raw_turtle;
  removeComments(raw_turtle);
  if(display_)
  {
    std::cout << file_name << std::endl;
    std::cout << "├── Individuals" << std::endl;
  }

  readTriplets(raw_turtle);

  if(previous_subject_ != "")
  {
    individual_graph_->add(previous_subject_, individual_vector_);
    nb_loaded_elem_++;
    individual_vector_ = IndividualVectors_t();
  }

  if(display_)
    std::cout << "└── "<< nb_loaded_elem_ << " readed ! " << std::endl;
  return NO_ERROR;
}

void OntologyTtlReader::removeComments(std::string& raw_turtle)
{
  char next_to_find = 0x00;
  bool is_multi_line = false;
  for(size_t i = 0; i < raw_turtle.size(); i++)
  {
    if(next_to_find == 0x00)
    {
      if(raw_turtle[i] == '<')
      {
        next_to_find = '>';
        i = raw_turtle.find(next_to_find, i + 1) - 1;
      }
      else if(raw_turtle[i] == '"')
      {
        next_to_find = '"';
        is_multi_line = isMultiLineDelimiter(raw_turtle, i, '"');
        i = raw_turtle.find(next_to_find, i + 1) - 1;
      }
      else if(raw_turtle[i] == '\'')
      {
        next_to_find = '\'';
        is_multi_line = isMultiLineDelimiter(raw_turtle, i, '\'');
        i = raw_turtle.find(next_to_find, i + 1) - 1;
      }
      else if(raw_turtle[i] == '#')
      {
        size_t j = raw_turtle.find('\n', i + 1);
        raw_turtle.erase(i, j-i);
        i--;
      }
    }
    else if(raw_turtle[i] == next_to_find)
    {
      if(is_multi_line == false)
      {
        if(isDelimiterEscaped(raw_turtle, i) == false)
          next_to_find = 0x00;
        else
          i = raw_turtle.find(next_to_find, i + 1) - 1;
      }
      else if(isMultiLineDelimiter(raw_turtle, i, next_to_find))
      {
        next_to_find = 0x00;
        is_multi_line = false;
      }
      else
        i = raw_turtle.find(next_to_find, i + 1) - 1;
    }
  }
}

void OntologyTtlReader::readTriplets(const std::string& raw_turtle)
{
  std::string current_subject;
  std::string current_property;
  std::string current_object;

  for(size_t i = 0; i < raw_turtle.size(); i++)
  {
    std::vector<std::array<std::string,3>> triplets;

    i = nextNonBlanckCharacter(raw_turtle, i);
    current_subject = getElement(raw_turtle, i);
    if(current_subject == "")
      break;

    bool repeated_property = false;
    do
    {
      if(repeated_property == false)
        i = nextNonBlanckCharacter(raw_turtle, i + current_subject.size() - 1);
      else
        i = nextNonBlanckCharacter(raw_turtle, i);
      current_property = getElement(raw_turtle, i);
      if(current_property == "")
      {
        Display::error("[Turtle parsing] fail to parse ttl file around the subject '" + current_subject + "'" );
        return;
      }

      bool repeated_object = false;
      do
      {
        if(repeated_object == false)
          i = nextNonBlanckCharacter(raw_turtle, i + current_property.size() - 1);
        else
          i = nextNonBlanckCharacter(raw_turtle, i);
        current_object = getElement(raw_turtle, i);
        if(current_object == "")
        {
          Display::error("[Turtle parsing] fail to parse ttl file around the object of the triplet '" + current_subject + " " + current_property + "'" );
          return;
        }

        triplets.emplace_back(std::array<std::string,3>{current_subject, current_property, current_object});

        i = nextNonBlanckCharacter(raw_turtle, i + current_object.size() - 1);

        switch (raw_turtle[i])
        {
        case '.':
          repeated_object = false;
          repeated_property = false;
          break;

        case ';':
          repeated_object = false;
          repeated_property = true;
          break;

        case ',':
          repeated_object = true;
          repeated_property = false;
          break;
        
        default:
          return;
        }
      }
      while (repeated_object == true);
      
    }
    while (repeated_property == true);

    sendToOntology(current_subject, triplets);
  }
}

void OntologyTtlReader::sendToOntology(std::string& subject, const std::vector<std::array<std::string,3>>& triplets)
{
  subject = getSubject(subject);
  if(subject != previous_subject_)
  {
    if(previous_subject_ != "")
    {
      individual_graph_->add(previous_subject_, individual_vector_);
      nb_loaded_elem_++;
      individual_vector_ = IndividualVectors_t();
    }
    previous_subject_ = subject;

    if(display_)
      std::cout << "│   ├──" << subject << std::endl;
  }

  for(auto& triplet : triplets)
  {
    auto object = getObject(triplet[2]);
    auto property = getProperty(triplet[1]);

    if(property == "rdf:type")
      push(individual_vector_.is_a_, object.first, 1.0, "+");
    else if(property == "owl:sameAs")
    {
      if( object.first != "")
        push(individual_vector_.same_as_, object.first, 1.0, "=");
    }
    else if(property == "rdfs:label")
      pushLang(individual_vector_.dictionary_, object);
    else if(property == "onto:label")
      pushLang(individual_vector_.muted_dictionary_, object);
    else if(object.second == "")
      OntologyReader::push(individual_vector_.object_relations_, Pair_t<std::string, std::string>(property, object.first, 1.0), "$", "^");
    else
    {
      LiteralNode data(object.second, object.first);
      OntologyReader::push(individual_vector_.data_relations_, Pair_t<std::string, std::string>(property, data.toString(), 1.0), "$", "^");
    }
  }
}

bool OntologyTtlReader::isMultiLineDelimiter(const std::string& raw_turtle, size_t& pose, char delim)
{
  size_t cpt = 0;
  while((raw_turtle[pose + cpt] == delim) && (cpt < 3))
    cpt++;

  if(cpt == 3)
  {
    pose += 2;
    return true;
  }
  else
    return false;
}


bool OntologyTtlReader::isDelimiterEscaped(const std::string& raw_turtle, size_t& pose)
{
  return (raw_turtle[pose - 1] == '\\');
}

size_t OntologyTtlReader::nextNonBlanckCharacter(const std::string& text, size_t pose)
{
  return text.find_first_not_of(" \n\r\t",pose+1);
}

size_t OntologyTtlReader::nextBlanckCharacter(const std::string& text, size_t pose)
{
  return text.find_first_of(" \n\r\t",pose+1);
}

size_t OntologyTtlReader::endOfBlock(const std::string& text, size_t pose)
{
  char next_to_find = 0x00;
  bool is_multi_line = false;

  char first_char = text[pose];
  if(first_char == '<')
    return text.find('>', pose+1);
  else if(first_char == '"')
  {
    next_to_find = '"';
    is_multi_line = isMultiLineDelimiter(text, pose, '"');
  }
  else if(first_char == '\'')
  {
    next_to_find = '\'';
    is_multi_line = isMultiLineDelimiter(text, pose, '\'');
  }
  else
    return std::string::npos;

  while(pose != std::string::npos)
  {
    pose = text.find(next_to_find, pose+1);
    if(pose == std::string::npos)
      return pose;
    else if(is_multi_line == false)
    {
      if(isDelimiterEscaped(text, pose) == false)
        return pose;
    }
    else if(isMultiLineDelimiter(text, pose, next_to_find))
      return pose;
  }

  return std::string::npos;
}

std::string OntologyTtlReader::getElement(const std::string& text, size_t pose)
{
  size_t end_pose;
  if(text[pose] == '<')
    end_pose = endOfBlock(text, pose);
  else if((text[pose] == '"') || (text[pose] == '\''))
  {
    end_pose = endOfBlock(text, pose);
    if((text[end_pose + 1] == '^') && (text[end_pose + 2] == '^'))
    {
      end_pose += 2;
      if(text[end_pose + 1] == '<')
        end_pose = endOfBlock(text, end_pose + 1);
      else if((text[pose] != '"') || (text[pose] != '\''))
        end_pose = nextBlanckCharacter(text, end_pose) - 1;
      else
        return "";
    }
    else if(text[end_pose + 1] == '@')
      end_pose = nextBlanckCharacter(text, end_pose) - 1;
  }
  else
    end_pose = nextBlanckCharacter(text, pose) - 1;

  if(end_pose != std::string::npos)
    return text.substr(pose, end_pose - pose + 1);
  else
    return "";
}

std::string OntologyTtlReader::getSubject(const std::string& element)
{
  if(element != "")
  {
    if(element[0] == '<')
    {
      size_t pose = element.rfind('/');
      size_t dash_pose = element.find('#', pose);
      if(dash_pose != std::string::npos)
        pose = dash_pose;

      if(pose != std::string::npos)
        return element.substr(pose + 1, element.size() - pose -2);
      else
        return element.substr(1, element.size() - 2);
    }
    else
    {
      size_t pose = element.rfind(':');
      if(pose != std::string::npos)
        return element.substr(pose + 1, element.size() - pose - 1);
      else
        return element;
    }
  }
  else
    return "";
}

std::string OntologyTtlReader::getProperty(const std::string& element)
{
  if(element[0] == '<')
  {
    if(element == "<http://www.w3.org/1999/02/22-rdf-syntax-ns#type>")
      return "rdf:type";
    else if(element == "<http://www.w3.org/2000/01/rdf-schema#label>")
      return "rdfs:label";
    else if(element == "<http://www.w3.org/2002/07/owl#sameAs>")
      return "owl:sameAs";
    else
      return getSubject(element);
  }
  else if((element == "a") || (element == "rdf:type"))
    return "rdf:type";
  else if(element == "rdfs:label")
    return "rdfs:label";
  else if(element == "owl:sameAs")
    return "owl:sameAs";
  else if(element == "onto:label")
    return element;
  else if(element == ":")
    return element;
  else
    return getSubject(element);
}

std::pair<std::string, std::string> OntologyTtlReader::getObject(const std::string& element)
{
  std::pair<std::string, std::string> object;

  if(element != "")
  {
    if(element[0] == '<')
    {
      size_t pose = element.rfind('/');
      size_t dash_pose = element.find('#', pose);
      if(dash_pose != std::string::npos)
        pose = dash_pose;

      if(pose != std::string::npos)
        object.first = element.substr(pose + 1, element.size() - pose -2);
      else
        object.first = element.substr(1, element.size() - 2);

      return object;
    }
    else if((element[0] == '"') || (element[0] == '\''))
    {
      size_t end_pose = endOfBlock(element, 0);
      object.first = element.substr(1, end_pose - 1); // todo manage multiline
      if(end_pose == element.size() - 1)
        object.second = "string";
      else if(element[end_pose + 1] == '@')
        object.second = element.substr(end_pose + 1);
      else if(element[end_pose + 1] == '^')
      {
        std::string type = element.substr(end_pose + 3);
        object.second = getSubject(type);
      }
    }
    else
    {
      size_t pose = element.rfind(':');
      if(pose != std::string::npos)
        object.first = element.substr(pose + 1, element.size() - pose - 1);
      else
        object.first = element;

      return object;
    }
  }

  if(object.second == "")
  {
    if((object.first == "true") || (object.first == "false"))
      object.second = "boolean";
    else
    {
      std::smatch match;
      if(std::regex_search(object.first, match, double_reg_))
        object.second = "double";
      else if(std::regex_search(object.first, match, decimal_reg_))
        object.second = "decimal";
      else if(std::regex_search(object.first, match, integer_reg_))
        object.second = "integer";
    }
  }

  return object;
}

void OntologyTtlReader::push(std::vector<Single_t<std::string>>& vect, const std::string& element, float probability, const std::string& symbole)
{
  vect.emplace_back(element, probability);
  if(display_ && symbole != "")
    std::cout << "│   │   ├── " << symbole << element << std::endl;
}

void OntologyTtlReader::pushLang(std::map<std::string, std::vector<std::string>>& dictionary, const std::pair<std::string, std::string>& label)
{
  std::string lang = "en";
  if((label.second != "") && (label.second != "string"))
    lang = label.second.substr(1);
  
  dictionary[lang].emplace_back(label.first);

  if(display_ && (label.first != ""))
    std::cout << "│   │   ├── " << "@" << lang << " : " << dictionary[lang][dictionary[lang].size() - 1] << std::endl;
}

} // namespace ontologenius