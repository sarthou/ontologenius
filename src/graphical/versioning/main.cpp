#include <cstddef>
#include <iostream>
#include <string>

#include "ontologenius/graphical/versioning/TreeDrawer.h"
#include "ontologenius/graphical/versioning/TreeReader.h"

std::string getFileName(const std::string& xml_path)
{
  std::string png_path = xml_path;

  size_t dot_pose = std::string::npos;
  size_t pose = std::string::npos;
  for(size_t i = 0; i < xml_path.size(); i++)
  {
    if(xml_path[i] == '.')
      dot_pose = i;
    else if(xml_path[i] == '/')
      pose = i;
  }

  if(pose == std::string::npos)
  {
    if(dot_pose == std::string::npos)
      png_path += ".png";
    else
      png_path.replace(dot_pose, png_path.size() - dot_pose, ".png");
  }
  else
  {
    if(dot_pose == std::string::npos)
      png_path += ".png";
    else if(dot_pose > pose)
      png_path.replace(dot_pose, png_path.size() - dot_pose, ".png");
    else
      png_path += ".png";
  }

  return png_path;
}

int main(int argc, char** argv)
{
  std::string path;
  bool commit_only = false;

  if(argc > 1)
    path = std::string(argv[1]);
  else
  {
    std::cout << "No input file. Pass it as argument." << std::endl;
    return -1;
  }

  for(int i = 2; i < argc; i++)
  {
    if(std::string(argv[i]) == "-c")
      commit_only = true;
  }

  ontologenius::TreeReader reader;
  ontologenius::Commit* commit = reader.read(path);

  std::cout << "width = " << ontologenius::Commit::global_width << std::endl;
  std::cout << "height = " << ontologenius::Commit::global_height << std::endl;

  ontologenius::TreeDrawer drawer;
  drawer.draw(getFileName(path), commit, commit_only);

  if(commit_only == false)
    std::cout << "--> You can get only the commit graph with the option -c <--" << std::endl;

  delete commit;

  return 0;
}
