#ifndef ONTOLOGENIUS_PARAMETERS_H
#define ONTOLOGENIUS_PARAMETERS_H

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ontologenius/graphical/Display.h"
#include "ontologenius/utils/String.h"

namespace ontologenius {

  class Parameter
  {
  public:
    std::string name_;
    std::vector<std::string> options_;
    std::vector<std::string> values_;
    std::vector<std::string> default_values_;

    Parameter(const std::string& name,
              const std::vector<std::string>& options,
              const std::vector<std::string>& default_values = {}) : name_(name),
                                                                     options_(options),
                                                                     default_values_(default_values)
    {}

    Parameter(const Parameter& other) = default;

    void insert(const std::string& value)
    {
      if(value.find("__") != 0)
        values_.push_back(value);
    }

    std::string getFirst()
    {
      if(values_.empty())
        return ((default_values_.empty() == false) ? default_values_[0] : "");
      else
        return ((values_.empty() == false) ? values_[0] : "");
    }

    std::vector<std::string> get() const
    {
      if(values_.empty())
        return default_values_;
      else
        return values_;
    }

    bool testOption(const std::string& option)
    {
      return std::any_of(options_.begin(), options_.end(), [option](auto op) { return option == op; });
    }

    void display() const
    {
      Display::info(name_ + ":");

      if(values_.empty() == false)
      {
        for(const auto& value : values_)
          Display::info("\t- " + value);
      }
      else
      {
        for(const auto& value : default_values_)
          Display::info("\t- " + value);
      }
    }
  };

  class Parameters
  {
  private:
    std::map<std::string, Parameter> parameters_;
    std::string default_param_name_;
    std::string process_name_;

  public:
    /// @brief Register a new parameter model in the parameter set
    /// @param param is the parameter object to insert
    void insert(const Parameter& param)
    {
      parameters_.insert(std::pair<std::string, Parameter>(param.name_, param));
      if(param.options_.empty())
        default_param_name_ = param.name_;
    }

    /// @brief Returns the parameter object related to the name provided in argument
    /// @param parameter is the name of the parameter to get
    /// @return A copy of the parameter object
    Parameter at(const std::string& parameter) const
    {
      return parameters_.at(parameter);
    }

    /// @brief Sets/Reads the values of the parameters
    /// @param argc the number of strings pointed to by argv
    /// @param argv is the array of arguments
    void set(int argc, char** argv)
    {
      process_name_ = std::string(argv[0]);
      size_t pose = 0;
      while((pose = process_name_.find('/')) != std::string::npos)
      {
        process_name_ = process_name_.substr(pose + 1);
      }
      process_name_ = " " + process_name_ + " ";

      for(size_t i = 1; i < (size_t)argc; i++)
      {
        std::string str_argv = std::string(argv[i]);
        if(str_argv[0] == '-')
        {
          if(str_argv == "--ros-args") // do not consider ROS arguments
            break;

          std::string param_name;
          for(auto param : parameters_)
            if(param.second.testOption(str_argv))
            {
              param_name = param.second.name_;
              break;
            }

          if(param_name.empty())
            Display::warning("unknow option " + str_argv);
          else
          {
            if(i + 1 < (size_t)argc)
            {
              i++;
              str_argv = std::string(argv[i]);
              auto splitted = split(str_argv, " ");
              for(auto& arg : splitted)
                if(arg.empty() == false)
                  parameters_.at(param_name).insert(arg);
            }
          }
        }
        else
        {
          if(default_param_name_.empty() == false)
          {
            auto splitted = split(str_argv, " ");
            for(auto& arg : splitted)
              if(arg.empty() == false)
                parameters_.at(default_param_name_).insert(arg);
          }
          else
            Display::warning("No default parameter");
        }
      }
    }

    /// @brief Displays the parameters names and setted values
    void display() const
    {
      std::string delim = "****************";
      std::string delim_gap;
      for(size_t i = 0; i < process_name_.size(); i++)
        delim_gap += "*";
      Display::info(delim + process_name_ + delim);
      for(const auto& param : parameters_)
        param.second.display();
      Display::info(delim + delim_gap + delim);
    }
  };

} // namespace ontologenius

#endif // ONTOLOGENIUS_PARAMETERS_H
