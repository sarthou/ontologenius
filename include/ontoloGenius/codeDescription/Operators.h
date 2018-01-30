#ifndef OPERATORS_H
#define OPERATORS_H

#include <string>
#include <vector>
#include <stdint.h>
#include <set>

struct Operator_t
{
  std::string op;
  std::string replace;
  size_t begin;
  size_t size;
  size_t end_bracket;
};

struct OperatorDescriptor_t
{
  std::string op;
  std::string function;
  bool whole_line;
  uint8_t priority;
  bool dont_carre;
};


class Operators
{
public:
  Operators(std::string* code) {code_ = code; }
  ~Operators() {}

  bool describe(std::string op, std::string function, bool whole_line, uint8_t priority);
  void dontCarre(std::string op);

  void op2Function();
  void function2Op(std::string& text, size_t pose, size_t& track_pose);

  size_t findNextOperator(size_t pose);
  OperatorDescriptor_t* isPreOperator(size_t& pose);
  OperatorDescriptor_t* isPostOperator(size_t pose);

private:
  std::string* code_;

  std::vector<Operator_t> operators_;
  std::vector<OperatorDescriptor_t> descriptors_;
};

#endif
