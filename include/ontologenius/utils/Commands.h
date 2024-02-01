#include <stdexcept>
#include <stdio.h>
#include <string>

#include "ontologenius/utils/String.h"

namespace ontologenius {

std::string execCmd(std::string cmd)
{
  char buffer[128];
  std::string result = "";
  cmd.append(" 2>&1");
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) throw std::runtime_error("popen() failed!");
  try
  {
    while (fgets(buffer, sizeof buffer, pipe) != NULL)
        result += buffer;
  }
  catch (...)
  {
    pclose(pipe);
    return "";
  }

  pclose(pipe);
  if(result.find("not found") != std::string::npos)
    return "";

  return result;
}

std::string findPackageRos1(const std::string& pkg_name)
{
  std::string results = execCmd("rospack find " + pkg_name);
  if(results.size())
  {
    auto split_res = split(results, "\n");
    return split_res.front();
  }
  else
    return "";
}

std::string findPackageRos2(const std::string& pkg_name)
{
  std::string results = execCmd("ros2 pkg prefix " + pkg_name);
  if(results.size())
  {
    auto split_res = split(results, "\n");
    return split_res.front() + "/share/" + pkg_name;
  }
  else
    return "";
}

std::string findPackage(const std::string& pkg_name)
{
  std::string res = findPackageRos1(pkg_name);
  if(res == "")
    res = findPackageRos2(pkg_name);
  return res;
}

std::vector<std::string> listPackagesRos1()
{
  std::string results = execCmd("rospack list-names");
  if(results.size())
  {
    auto split_res = split(results, "\n");
    return split_res;
  }
  else
    return {};
}

std::vector<std::string> listPackagesRos2()
{
  std::string results = execCmd("ros2 pkg list");
  if(results.size())
  {
    auto split_res = split(results, "\n");
    return split_res;
  }
  else
    return {};
}

std::vector<std::string> listPackages()
{
  std::vector<std::string> res = listPackagesRos1();
  if(res.size() == 0)
    res = listPackagesRos2();
  return res;
}

} // namespace ontologenius