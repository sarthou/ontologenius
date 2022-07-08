#ifndef ONTOLOGENIUS_SPARQLSOLVER_H
#define ONTOLOGENIUS_SPARQLSOLVER_H

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/ontologyOperators/SparqlUtils.h"

#include <regex>
#include <unordered_set>

namespace ontologenius
{

struct SparqlConstraint_t
{
  triplet_t triplet_;
  SparqlOperator_e operator_;
  bool subject_constraint_;
  bool leaf_;
};

struct SparqlVariableConstraint_t
{
  SparqlVariableConstraint_t(const std::string& variable) : variable_(variable) {}
  std::string variable_;
  std::vector<SparqlConstraint_t> constraints_;
  std::unordered_set<std::string> linked_variales_;
};

struct SparqlSolution_t
{
  std::map<std::string, std::string> solution_;
  std::map<std::string, std::string> solution_full_;
  std::map<std::string, SparqlVariableConstraint_t> variable_constraints_;
  std::vector<std::string> ordered_variables_;
  std::map<std::string, std::unordered_set<std::string>> candidates_;
};

class SparqlSolver
{
public:
  SparqlSolver();
  void link(Ontology* onto) { onto_ = onto; }
  void set(const std::string& query) { query_ = query; }

  std::string getError() { return error_; }

  class iterator
  {
    public:
      explicit iterator(const SparqlSolution_t& intial_solution, SparqlSolver* solver = nullptr) : current_solution_(intial_solution)
      {
        solver_ = solver;
      }
      iterator(const iterator& other) = default;

      operator bool() const { return !is_empty(); }

      bool operator==(const iterator& other)const{ return (current_solution_.solution_full_ == other.current_solution_.solution_full_); }
      bool operator!=(const iterator& other)const{ return (current_solution_.solution_full_ != other.current_solution_.solution_full_); }

      std::map<std::string, std::string>& operator*()
      { 
        for(auto& sol_it : current_solution_.solution_)
          current_solution_.solution_[sol_it.first] = current_solution_.solution_full_[sol_it.first];
        return current_solution_.solution_;
      }

      iterator& operator++() { stepForward(); return (*this); }

    private:
      SparqlSolution_t current_solution_;
      SparqlSolver* solver_;

      bool is_empty() const
      {
        if(std::any_of(current_solution_.solution_full_.begin(), current_solution_.solution_full_.end(), [](auto it){ return it.second != ""; }))
          return false;
        return true;
      }

      void stepForward()
      {
        if(solver_ == nullptr)
        {
          for(auto& it : current_solution_.solution_full_)
            it.second = "";
        }
        else
          solver_->nextSolution(current_solution_);
      }
  };

  iterator begin();
  const iterator cbegin();
  iterator end() { return iterator(empty_solution_); }
  const iterator cend() { return iterator(empty_solution_); }

private:
  ontologenius::Ontology* onto_;
  std::regex sparql_pattern_;
  std::map<std::string, SparqlOperator_e> operators_;

  std::string error_;
  std::string query_;
  SparqlSolution_t empty_solution_;

  SparqlSolution_t getInitialSolutionStandard(const std::string& pattern);
  SparqlSolution_t getInitialSolutionCustom();
  void insertConstraints(const std::vector<triplet_t>& triplets, SparqlSolution_t& solution, SparqlOperator_e sparql_operator);
  void orderVariables(SparqlSolution_t& solution);
  void stepDown(SparqlSolution_t& solution, int index);
  void nextSolution(SparqlSolution_t& solution);

  std::unordered_set<std::string> solveTriplet(triplet_t triplet, const std::map<std::string, std::string>& binding);
  std::unordered_set<std::string> getOn(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getFrom(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getUp(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getType(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> find(const triplet_t& triplet, const std::string& selector = "");
  std::unordered_set<std::string> getName(const triplet_t& triplet, const std::string& selector = "");

  std::string getPattern(const std::string& text);
  std::vector<SparqlBlock_t> getBlocks(std::string query);
  std::vector<triplet_t> getTriplets(const std::string& query, const std::string& delim);
};

} // namespace ontologenius

#endif // ONTOLOGENIUS_SPARQLSOLVER_H
